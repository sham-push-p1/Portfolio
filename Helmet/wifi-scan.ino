// ============================================================
// MineGuard - Smart Mining Safety Helmet (Wokwi simulation)
// ESP32 DevKit C v4
//
// Core 1 (Arduino loop): gas / DHT / MPU / strap / SOS sampling,
//                        4-level alert engine, buzzer + motor
// Core 0 (net task)    : vitals (HR/SpO2) + Wi-Fi posting
//
// Stand-ins for Wokwi:
//   MAX30102 -> two potentiometers (GPIO36 = HR, GPIO39 = SpO2)
//   Reed switch (strap) -> slide switch on GPIO27
//   MQ-135 is used as the NH3 channel
// RFID checkpoints are a separate project (checkpoint nodes).
// ============================================================

#include <Wire.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <DHT.h>
#include <math.h>

// ---------------- Config ----------------
#define DEVICE_ID        "helmet-01"
#define WIFI_SSID        "Wokwi-GUEST"
#define WIFI_PASS        ""
#define WIFI_CHANNEL     6
#define SERVER_URL       "http://example.com/api/readings"   // TODO: your backend endpoint
#define POST_INTERVAL_MS 2000

// ---------------- Pins ----------------
#define PIN_MQ7_CO     34   // ADC1_CH6
#define PIN_MQ135_NH3  35   // ADC1_CH7
#define PIN_MQ4_CH4    32   // ADC1_CH4
#define PIN_POT_HR     36   // simulated MAX30102 heart rate
#define PIN_POT_SPO2   39   // simulated MAX30102 SpO2
#define PIN_DHT        15
#define PIN_SDA        21
#define PIN_SCL        22
#define PIN_BUZZER     25
#define PIN_MOTOR      26
#define PIN_SOS_BTN    13   // to GND when pressed
#define PIN_STRAP      27   // reed switch stand-in

#define DHT_TYPE            DHT22   // Wokwi part is a DHT22; use DHT11 on real hardware
#define STRAP_CLOSED_LEVEL  LOW     // flip to HIGH if your switch reads inverted

// ---------------- Alert levels ----------------
enum Level { SAFE = 0, WARNING = 1, DANGER = 2, CRITICAL = 3 };
const char *LEVEL_NAME[] = { "SAFE", "WARNING", "DANGER", "CRITICAL" };

// ---------------- Thresholds (tune these) ----------------
// Gas: raw 12-bit ADC (0-4095). Real sensors: calibrate against clean-air baseline.
//                 warn  danger critical
const float CO_T[3]  = { 1200, 2000, 3000 };
const float NH3_T[3] = { 1200, 2000, 3000 };
const float CH4_T[3] = { 1000, 1800, 2800 };

// Rate of rise over the last <=30 s (raw ADC counts)
const int RISE_WARN   = 400;
const int RISE_DANGER = 800;
const int RISE_WINDOW_S = 30;

// Heart rate depends on motion. Low side (<=) and high side (>=).
//                       lowW lowD lowC  highW highD highC
const float HR_STILL[6]  = { 50, 45, 40,   100, 120, 150 };
const float HR_MOVING[6] = { 55, 50, 45,   140, 160, 180 };

// SpO2 (<=): warn, danger, critical
const float SPO2_T[3] = { 93, 89, 84 };

// Ambient temperature (C)
const float TEMP_T[3] = { 35, 40, 45 };

// Motion detection (deviation of |accel| from 1 g, smoothed)
const float MOVE_ON_G  = 0.06;
const float MOVE_OFF_G = 0.03;

// Escalation rules
const unsigned long WARNING_ESCALATE_MS = 120000UL;  // Warning held 120 s -> Danger
const unsigned long SOS_LATCH_MS        = 30000UL;   // SOS stays active 30 s after press

// ---------------- Shared state (guarded by mutex) ----------------
struct Snapshot {
  int   co, nh3, ch4;
  float temp, hum;
  int   bpm, spo2;
  bool  moving, strapped, sos;
  int   level;
  char  reason[96];
};
Snapshot S;
SemaphoreHandle_t mtx;
#define LOCK()   xSemaphoreTake(mtx, portMAX_DELAY)
#define UNLOCK() xSemaphoreGive(mtx)

// ---------------- Core-1 private state ----------------
DHT dht(PIN_DHT, DHT_TYPE);
const uint8_t MPU_ADDR = 0x68;

struct RiseBuf {
  int v[RISE_WINDOW_S];
  int idx = 0;
  int count = 0;
  void push(int x) {
    v[idx] = x;
    idx = (idx + 1) % RISE_WINDOW_S;
    if (count < RISE_WINDOW_S) count++;
  }
  int rise(int now) {
    if (count < 10) return 0;   // need at least ~10 s of history
    int oldestIdx = (idx - count + RISE_WINDOW_S) % RISE_WINDOW_S;
    return now - v[oldestIdx];
  }
};
RiseBuf riseCo, riseNh3, riseCh4;

int   rawCo = 0, rawNh3 = 0, rawCh4 = 0;
float tempC = 25, humP = 50;
bool  moving = false, strapped = true, sosActive = false;
float motionEma = 0;
unsigned long sosUntil = 0;
bool lastSosPin = HIGH;

unsigned long warnSince = 0;
Level currentLevel = SAFE;

// ---------------- Helpers ----------------
int readAdcAvg(int pin, int n = 8) {
  long sum = 0;
  for (int i = 0; i < n; i++) sum += analogRead(pin);
  return sum / n;
}

int scoreHigh(float v, const float t[3]) {
  if (v >= t[2]) return 3;
  if (v >= t[1]) return 2;
  if (v >= t[0]) return 1;
  return 0;
}
int scoreLow(float v, const float t[3]) {
  if (v <= t[2]) return 3;
  if (v <= t[1]) return 2;
  if (v <= t[0]) return 1;
  return 0;
}
int riseScore(int rise) {
  if (rise >= RISE_DANGER) return 2;
  if (rise >= RISE_WARN)   return 1;
  return 0;
}

// ---------------- MPU6050 (raw I2C) ----------------
void mpuInit() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission(true);
}

float mpuReadAccelG() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);
  if (Wire.endTransmission(false) != 0) return 1.0f;
  if (Wire.requestFrom((int)MPU_ADDR, 6, (int)true) != 6) return 1.0f;
  int16_t ax = (Wire.read() << 8) | Wire.read();
  int16_t ay = (Wire.read() << 8) | Wire.read();
  int16_t az = (Wire.read() << 8) | Wire.read();
  float gx = ax / 16384.0f, gy = ay / 16384.0f, gz = az / 16384.0f;
  return sqrtf(gx * gx + gy * gy + gz * gz);
}

// ---------------- Fast sampling (100 ms) ----------------
void sampleFast() {
  rawCo  = readAdcAvg(PIN_MQ7_CO);
  rawNh3 = readAdcAvg(PIN_MQ135_NH3);
  rawCh4 = readAdcAvg(PIN_MQ4_CH4);

  strapped = (digitalRead(PIN_STRAP) == STRAP_CLOSED_LEVEL);

  // SOS: latch for SOS_LATCH_MS on each press
  bool pin = digitalRead(PIN_SOS_BTN);
  if (pin == LOW && lastSosPin == HIGH) sosUntil = millis() + SOS_LATCH_MS;
  lastSosPin = pin;
  sosActive = (pin == LOW) || (millis() < sosUntil);

  // Motion vs stationary (with hysteresis)
  float dyn = fabsf(mpuReadAccelG() - 1.0f);
  motionEma = 0.9f * motionEma + 0.1f * dyn;
  if (!moving && motionEma > MOVE_ON_G)  moving = true;
  if (moving  && motionEma < MOVE_OFF_G) moving = false;
}

void sampleDht() {
  float t = dht.readTemperature();
  float h = dht.readHumidity();
  if (!isnan(t)) tempC = t;
  if (!isnan(h)) humP  = h;
}

// ---------------- Alert engine ----------------
void evaluate() {
  unsigned long now = millis();

  // Feed rate-of-rise buffers once per second
  static unsigned long lastPush = 0;
  if (now - lastPush >= 1000) {
    lastPush = now;
    riseCo.push(rawCo);
    riseNh3.push(rawNh3);
    riseCh4.push(rawCh4);
  }

  int bpm, spo2;
  LOCK(); bpm = S.bpm; spo2 = S.spo2; UNLOCK();

  // Per-reading scores (0 safe, 1 warning, 2 danger, 3 critical)
  int sCo  = max(scoreHigh(rawCo,  CO_T),  riseScore(riseCo.rise(rawCo)));
  int sNh3 = max(scoreHigh(rawNh3, NH3_T), riseScore(riseNh3.rise(rawNh3)));
  int sCh4 = max(scoreHigh(rawCh4, CH4_T), riseScore(riseCh4.rise(rawCh4)));

  const float *hr = moving ? HR_MOVING : HR_STILL;
  const float hrLow[3]  = { hr[0], hr[1], hr[2] };
  const float hrHigh[3] = { hr[3], hr[4], hr[5] };
  int sHr   = max(scoreLow(bpm, hrLow), scoreHigh(bpm, hrHigh));
  int sSpo2 = scoreLow(spo2, SPO2_T);
  int sTemp = scoreHigh(tempC, TEMP_T);

  struct { const char *name; int score; } ch[] = {
    { "CO", sCo }, { "NH3", sNh3 }, { "CH4", sCh4 },
    { "HR", sHr }, { "SpO2", sSpo2 }, { "TEMP", sTemp }
  };

  int maxScore = 0, warnCount = 0, dangerCount = 0;
  char reason[96] = "";
  for (auto &c : ch) {
    if (c.score > maxScore) maxScore = c.score;
    if (c.score == 1) warnCount++;
    if (c.score >= 2) dangerCount++;
    if (c.score >= 1) {
      char tmp[24];
      snprintf(tmp, sizeof(tmp), "%s%s:%s", reason[0] ? "," : "", c.name, LEVEL_NAME[c.score]);
      strncat(reason, tmp, sizeof(reason) - strlen(reason) - 1);
    }
  }

  // Overall level = max score, bumped by how many channels are elevated
  int overall = maxScore;
  if (overall == WARNING && warnCount >= 3)   overall = DANGER;
  if (overall == DANGER  && dangerCount >= 2) overall = CRITICAL;

  // Warning stuck too long -> Danger
  if (overall == WARNING) {
    if (warnSince == 0) warnSince = now;
    if (now - warnSince >= WARNING_ESCALATE_MS) {
      overall = DANGER;
      strncat(reason, ",WARN>120s", sizeof(reason) - strlen(reason) - 1);
    }
  } else if (overall == SAFE) {
    warnSince = 0;
  }

  // Absolute overrides
  if (!strapped) {
    if (overall < DANGER) overall = DANGER;
    strncat(reason, reason[0] ? ",STRAP_OPEN" : "STRAP_OPEN", sizeof(reason) - strlen(reason) - 1);
  }
  if (sosActive) {
    overall = CRITICAL;
    strncat(reason, reason[0] ? ",SOS" : "SOS", sizeof(reason) - strlen(reason) - 1);
  }

  currentLevel = (Level)overall;

  LOCK();
  S.co = rawCo; S.nh3 = rawNh3; S.ch4 = rawCh4;
  S.temp = tempC; S.hum = humP;
  S.moving = moving; S.strapped = strapped; S.sos = sosActive;
  S.level = overall;
  strncpy(S.reason, reason, sizeof(S.reason));
  S.reason[sizeof(S.reason) - 1] = 0;
  UNLOCK();
}

// ---------------- Outputs ----------------
// Motor (worker feels he is in a danger zone): Warning slow pulse, Danger fast pulse, Critical solid.
// Buzzer (emergency): silent at Safe/Warning, Danger beeping, Critical rapid.
void driveOutputs(Level lvl) {
  unsigned long now = millis();
  bool motorOn = false;
  bool buzzOn = false;
  int freq = 0;

  switch (lvl) {
    case SAFE: break;
    case WARNING:
      motorOn = (now % 1500) < 150;
      break;
    case DANGER:
      motorOn = (now % 500) < 200;
      buzzOn  = (now % 600) < 300;
      freq = 2200;
      break;
    case CRITICAL:
      motorOn = true;
      buzzOn  = (now % 250) < 125;
      freq = 3200;
      break;
  }

  digitalWrite(PIN_MOTOR, motorOn ? HIGH : LOW);
  if (buzzOn) tone(PIN_BUZZER, freq);
  else noTone(PIN_BUZZER);
}

// ---------------- Core 0: vitals + Wi-Fi ----------------
void postSnapshot(const Snapshot &c) {
  char body[420];
  snprintf(body, sizeof(body),
    "{\"device_id\":\"%s\",\"uptime_ms\":%lu,\"level\":%d,\"level_name\":\"%s\","
    "\"gas\":{\"co\":%d,\"nh3\":%d,\"ch4\":%d},"
    "\"vitals\":{\"bpm\":%d,\"spo2\":%d},"
    "\"env\":{\"temp\":%.1f,\"humidity\":%.0f},"
    "\"moving\":%s,\"strapped\":%s,\"sos\":%s,\"reason\":\"%s\"}",
    DEVICE_ID, millis(), c.level, LEVEL_NAME[c.level],
    c.co, c.nh3, c.ch4, c.bpm, c.spo2, c.temp, c.hum,
    c.moving ? "true" : "false", c.strapped ? "true" : "false",
    c.sos ? "true" : "false", c.reason);

  HTTPClient http;
  http.setTimeout(3000);
  if (http.begin(SERVER_URL)) {
    http.addHeader("Content-Type", "application/json");
    int code = http.POST((uint8_t *)body, strlen(body));
    Serial.printf("[net] POST -> %d\n", code);
    http.end();
  }
}

void netTask(void *) {
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS, WIFI_CHANNEL);

  unsigned long lastVital = 0, lastPost = 0, lastRetry = 0;
  int lastPostedLevel = -1;

  for (;;) {
    unsigned long now = millis();

    // Simulated MAX30102 (replace with real sensor code on hardware)
    if (now - lastVital >= 200) {
      lastVital = now;
      int bpm  = map(readAdcAvg(PIN_POT_HR),   0, 4095, 30, 200);
      int spo2 = map(readAdcAvg(PIN_POT_SPO2), 0, 4095, 75, 100);
      LOCK(); S.bpm = bpm; S.spo2 = spo2; UNLOCK();
    }

    bool connected = (WiFi.status() == WL_CONNECTED);
    if (!connected && now - lastRetry > 10000) {
      lastRetry = now;
      WiFi.disconnect();
      WiFi.begin(WIFI_SSID, WIFI_PASS, WIFI_CHANNEL);
    }

    Snapshot copy;
    LOCK(); copy = S; UNLOCK();

    bool changed = (copy.level != lastPostedLevel);
    if (connected && (changed || now - lastPost >= POST_INTERVAL_MS)) {
      lastPost = now;
      lastPostedLevel = copy.level;
      postSnapshot(copy);
    }

    vTaskDelay(pdMS_TO_TICKS(20));
  }
}

// ---------------- Arduino ----------------
void setup() {
  Serial.begin(115200);

  pinMode(PIN_BUZZER, OUTPUT);
  pinMode(PIN_MOTOR, OUTPUT);
  pinMode(PIN_SOS_BTN, INPUT_PULLUP);
  pinMode(PIN_STRAP, INPUT_PULLUP);
  analogReadResolution(12);

  Wire.begin(PIN_SDA, PIN_SCL);
  mpuInit();
  dht.begin();

  mtx = xSemaphoreCreateMutex();
  memset(&S, 0, sizeof(S));
  S.bpm = 75; S.spo2 = 98; S.temp = 25; S.hum = 50; S.strapped = true;

  // Core 0: vitals + Wi-Fi. loop() already runs on core 1.
  xTaskCreatePinnedToCore(netTask, "net", 8192, NULL, 1, NULL, 0);

  Serial.println("MineGuard helmet started");
}

void loop() {
  unsigned long now = millis();

  static unsigned long lastFast = 0, lastDht = 0, lastEval = 0, lastPrint = 0;

  if (now - lastFast >= 100) { lastFast = now; sampleFast(); }
  if (now - lastDht >= 2000) { lastDht = now; sampleDht(); }
  if (now - lastEval >= 200) { lastEval = now; evaluate(); }

  driveOutputs(currentLevel);

  if (now - lastPrint >= 1000) {
    lastPrint = now;
    Snapshot c;
    LOCK(); c = S; UNLOCK();
    Serial.printf("[%s] CO:%d NH3:%d CH4:%d | HR:%d SpO2:%d %s | T:%.1f H:%.0f | strap:%s sos:%s | %s\n",
      LEVEL_NAME[c.level], c.co, c.nh3, c.ch4, c.bpm, c.spo2,
      c.moving ? "moving" : "still", c.temp, c.hum,
      c.strapped ? "on" : "OPEN", c.sos ? "YES" : "no", c.reason);
  }

  delay(2);
}
