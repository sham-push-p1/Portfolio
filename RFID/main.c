// ============================================================
// MineGuard - RFID Checkpoint Node (Wokwi simulation)
// ESP32 DevKit C v4
//
// Real hardware: 1 ESP32 + 2x MFRC522 (reader A = outer, reader B = inner).
// Wokwi stand-ins:
//   Reader A  -> pushbutton on GPIO32  (reads the currently selected tag)
//   Reader B  -> pushbutton on GPIO33
//   Tag select-> pushbutton on GPIO25  (cycles through preset worker tags)
//   Serial monitor also works:  "a" / "b"   = trigger reader A / B
//                               "tag XXXX"  = set current tag ID
//
// Direction: A then B within PAIR_WINDOW_MS = ENTRY, B then A = EXIT.
// Outputs (LEDs): green = entry event, red = exit event,
//                 yellow = ignored (duplicate / cooldown) or timed out.
//
// Core 1 (loop): inputs + direction matching
// Core 0 (net) : Wi-Fi + one clean JSON POST per event (queue, retry)
// ============================================================

#include <WiFi.h>
#include <HTTPClient.h>
#include <time.h>

// ---------------- Config ----------------
#define CHECKPOINT_ID      "cp-01"     // change per checkpoint (cp-02, cp-03)
#define WIFI_SSID          "Wokwi-GUEST"
#define WIFI_PASS          ""
#define WIFI_CHANNEL       6
#define SERVER_URL         "http://example.com/api/checkpoint-events"  // TODO: your backend

#define PAIR_WINDOW_MS     4000   // max gap between reader A and reader B reads
#define READER_DEBOUNCE_MS 600    // same tag, same reader: ignore repeats inside this
#define COOLDOWN_MS        1500   // after an event, ignore that tag briefly
#define MAX_TAGS           8
#define MAX_POST_ATTEMPTS  3

// ---------------- Pins ----------------
#define PIN_BTN_A      32
#define PIN_BTN_B      33
#define PIN_BTN_TAG    25
#define PIN_LED_ENTRY  26
#define PIN_LED_EXIT   27
#define PIN_LED_IGN    14

const char *PRESET_TAGS[] = { "04A1B2C3", "04D4E5F6", "04070809" };
const int N_PRESET = sizeof(PRESET_TAGS) / sizeof(PRESET_TAGS[0]);
int  presetIdx = 0;
char currentTag[16];

// ---------------- Direction-matching state ----------------
// One slot per tag so two workers close together do not confuse each other.
struct TagInfo {
  bool used = false;
  char tag[16] = "";
  bool seen[2] = { false, false };
  unsigned long lastRead[2] = { 0, 0 };
  bool pending = false;
  uint8_t firstReader = 0;
  unsigned long firstAt = 0;
  bool hadEvent = false;
  unsigned long lastEvent = 0;
};
TagInfo tags[MAX_TAGS];

// ---------------- Event queue (to net task) ----------------
struct Event {
  char tag[16];
  char type[6];          // "ENTRY" / "EXIT"
  unsigned long uptime;
  time_t epoch;          // 0 if NTP not synced
};
QueueHandle_t evq;

// ---------------- LEDs ----------------
const uint8_t LED_PINS[3] = { PIN_LED_ENTRY, PIN_LED_EXIT, PIN_LED_IGN };
unsigned long ledOff[3] = { 0, 0, 0 };
enum { LED_ENTRY = 0, LED_EXIT = 1, LED_IGN = 2 };

void pulse(int i, unsigned long ms) {
  digitalWrite(LED_PINS[i], HIGH);
  ledOff[i] = millis() + ms;
}
void updateLeds() {
  for (int i = 0; i < 3; i++) {
    if (ledOff[i] && (long)(millis() - ledOff[i]) >= 0) {
      digitalWrite(LED_PINS[i], LOW);
      ledOff[i] = 0;
    }
  }
}

// ---------------- Buttons ----------------
struct Btn { uint8_t pin; bool state; bool raw; unsigned long t; };
Btn btnA   = { PIN_BTN_A,   HIGH, HIGH, 0 };
Btn btnB   = { PIN_BTN_B,   HIGH, HIGH, 0 };
Btn btnTag = { PIN_BTN_TAG, HIGH, HIGH, 0 };

bool pressedEdge(Btn &b) {
  bool r = digitalRead(b.pin);
  unsigned long now = millis();
  if (r != b.raw) { b.raw = r; b.t = now; }
  if (now - b.t >= 30 && r != b.state) {
    b.state = r;
    if (r == LOW) return true;
  }
  return false;
}

// ---------------- Tag table helpers ----------------
TagInfo *getInfo(const char *tag) {
  for (int i = 0; i < MAX_TAGS; i++)
    if (tags[i].used && strcmp(tags[i].tag, tag) == 0) return &tags[i];
  for (int i = 0; i < MAX_TAGS; i++)
    if (!tags[i].used) {
      tags[i] = TagInfo();
      tags[i].used = true;
      strncpy(tags[i].tag, tag, sizeof(tags[i].tag) - 1);
      return &tags[i];
    }
  // Table full: reuse a slot with nothing pending
  for (int i = 0; i < MAX_TAGS; i++)
    if (!tags[i].pending) {
      tags[i] = TagInfo();
      tags[i].used = true;
      strncpy(tags[i].tag, tag, sizeof(tags[i].tag) - 1);
      return &tags[i];
    }
  return &tags[0];
}

void emitEvent(const char *tag, const char *type) {
  Event e;
  memset(&e, 0, sizeof(e));
  strncpy(e.tag, tag, sizeof(e.tag) - 1);
  strncpy(e.type, type, sizeof(e.type) - 1);
  e.uptime = millis();
  time_t t = time(nullptr);
  e.epoch = (t > 1700000000) ? t : 0;
  if (xQueueSend(evq, &e, 0) != pdTRUE) Serial.println("[cp] queue full, event dropped");
}

// ---------------- Core logic: one reader read ----------------
void handleRead(uint8_t reader, const char *tag) {
  unsigned long now = millis();
  const char R = reader == 0 ? 'A' : 'B';
  TagInfo *ti = getInfo(tag);

  // Cooldown after a completed event
  if (ti->hadEvent && now - ti->lastEvent < COOLDOWN_MS) {
    Serial.printf("[cp] reader %c tag %s ignored (cooldown)\n", R, tag);
    pulse(LED_IGN, 150);
    return;
  }

  // Duplicate read on the same reader
  if (ti->seen[reader] && now - ti->lastRead[reader] < READER_DEBOUNCE_MS) {
    Serial.printf("[cp] reader %c tag %s ignored (duplicate read)\n", R, tag);
    pulse(LED_IGN, 150);
    return;
  }
  ti->seen[reader] = true;
  ti->lastRead[reader] = now;

  if (!ti->pending) {
    ti->pending = true;
    ti->firstReader = reader;
    ti->firstAt = now;
    Serial.printf("[cp] reader %c saw %s, waiting for reader %c\n", R, tag, reader == 0 ? 'B' : 'A');
    return;
  }

  // Same reader again before the other one fired: just refresh the window
  if (ti->firstReader == reader) {
    ti->firstAt = now;
    Serial.printf("[cp] reader %c saw %s again, window refreshed\n", R, tag);
    return;
  }

  // Pair complete: A->B = ENTRY, B->A = EXIT
  const char *type = (ti->firstReader == 0) ? "ENTRY" : "EXIT";
  ti->pending = false;
  ti->hadEvent = true;
  ti->lastEvent = now;
  Serial.printf("[cp] %s  tag=%s  checkpoint=%s\n", type, tag, CHECKPOINT_ID);
  pulse(strcmp(type, "ENTRY") == 0 ? LED_ENTRY : LED_EXIT, 500);
  emitEvent(tag, type);
}

// Drop single reads that never got a matching second read
void expirePending() {
  unsigned long now = millis();
  for (int i = 0; i < MAX_TAGS; i++) {
    if (tags[i].used && tags[i].pending && now - tags[i].firstAt > PAIR_WINDOW_MS) {
      tags[i].pending = false;
      Serial.printf("[cp] timeout: tag %s only seen by reader %c, discarded\n",
                    tags[i].tag, tags[i].firstReader == 0 ? 'A' : 'B');
      pulse(LED_IGN, 400);
    }
  }
}

// ---------------- Serial commands ----------------
void handleSerialLine(char *line) {
  // trim
  int n = strlen(line);
  while (n > 0 && (line[n - 1] == '\r' || line[n - 1] == ' ')) line[--n] = 0;
  if (n == 0) return;

  if (strcmp(line, "a") == 0 || strcmp(line, "A") == 0) {
    handleRead(0, currentTag);
  } else if (strcmp(line, "b") == 0 || strcmp(line, "B") == 0) {
    handleRead(1, currentTag);
  } else if (strncmp(line, "tag ", 4) == 0 && strlen(line + 4) > 0) {
    strncpy(currentTag, line + 4, sizeof(currentTag) - 1);
    currentTag[sizeof(currentTag) - 1] = 0;
    Serial.printf("[cp] current tag = %s\n", currentTag);
  } else {
    Serial.println("[cp] commands: a | b | tag <ID>");
  }
}

void pollSerial() {
  static char buf[32];
  static int len = 0;
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n') {
      buf[len] = 0;
      handleSerialLine(buf);
      len = 0;
    } else if (len < (int)sizeof(buf) - 1) {
      buf[len++] = c;
    }
  }
}

// ---------------- Core 0: Wi-Fi + posting ----------------
int postEvent(const Event &e) {
  char body[256];
  char ts[24];
  if (e.epoch) snprintf(ts, sizeof(ts), "%lu", (unsigned long)e.epoch);
  else strcpy(ts, "null");
  snprintf(body, sizeof(body),
    "{\"tag_id\":\"%s\",\"checkpoint_id\":\"%s\",\"event_type\":\"%s\","
    "\"timestamp\":%s,\"uptime_ms\":%lu}",
    e.tag, CHECKPOINT_ID, e.type, ts, e.uptime);

  HTTPClient http;
  http.setTimeout(3000);
  int code = -1;
  if (http.begin(SERVER_URL)) {
    http.addHeader("Content-Type", "application/json");
    code = http.POST((uint8_t *)body, strlen(body));
    http.end();
  }
  Serial.printf("[net] POST %s -> %d\n", body, code);
  return code;
}

void netTask(void *) {
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS, WIFI_CHANNEL);

  bool ntpStarted = false;
  unsigned long lastRetry = 0;
  int attempts = 0;

  for (;;) {
    unsigned long now = millis();
    bool connected = (WiFi.status() == WL_CONNECTED);

    if (connected && !ntpStarted) {
      configTime(0, 0, "pool.ntp.org", "time.nist.gov");   // UTC epoch
      ntpStarted = true;
      Serial.println("[net] Wi-Fi connected, NTP started");
    }
    if (!connected && now - lastRetry > 10000) {
      lastRetry = now;
      WiFi.disconnect();
      WiFi.begin(WIFI_SSID, WIFI_PASS, WIFI_CHANNEL);
    }

    Event e;
    if (xQueuePeek(evq, &e, pdMS_TO_TICKS(200)) == pdTRUE) {
      if (!connected) {
        vTaskDelay(pdMS_TO_TICKS(500));      // keep event, wait for Wi-Fi
      } else {
        int code = postEvent(e);
        if (code >= 200 && code < 300) {
          xQueueReceive(evq, &e, 0);         // delivered
          attempts = 0;
        } else if (++attempts >= MAX_POST_ATTEMPTS) {
          xQueueReceive(evq, &e, 0);         // give up on this one
          Serial.println("[net] event dropped after retries");
          attempts = 0;
        } else {
          vTaskDelay(pdMS_TO_TICKS(2000));   // retry the same event
        }
      }
    }
  }
}

// ---------------- Arduino ----------------
void setup() {
  Serial.begin(115200);

  pinMode(PIN_BTN_A, INPUT_PULLUP);
  pinMode(PIN_BTN_B, INPUT_PULLUP);
  pinMode(PIN_BTN_TAG, INPUT_PULLUP);
  for (int i = 0; i < 3; i++) {
    pinMode(LED_PINS[i], OUTPUT);
    digitalWrite(LED_PINS[i], LOW);
  }

  strncpy(currentTag, PRESET_TAGS[0], sizeof(currentTag) - 1);
  currentTag[sizeof(currentTag) - 1] = 0;

  evq = xQueueCreate(16, sizeof(Event));
  xTaskCreatePinnedToCore(netTask, "net", 8192, NULL, 1, NULL, 0);

  Serial.printf("MineGuard checkpoint %s started\n", CHECKPOINT_ID);
  Serial.printf("[cp] current tag = %s  (button cycles tags; serial: a | b | tag <ID>)\n", currentTag);
}

void loop() {
  if (pressedEdge(btnTag)) {
    presetIdx = (presetIdx + 1) % N_PRESET;
    strncpy(currentTag, PRESET_TAGS[presetIdx], sizeof(currentTag) - 1);
    Serial.printf("[cp] current tag = %s\n", currentTag);
  }
  if (pressedEdge(btnA)) handleRead(0, currentTag);
  if (pressedEdge(btnB)) handleRead(1, currentTag);

  pollSerial();
  expirePending();
  updateLeds();

  delay(2);
}
