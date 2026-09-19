/*
  Foot Monitor - Receiver / Plotter
  Flash this to the "base station" board (not the one on the shoe).

  Listens for FootPacket data over ESP-NOW from the foot tracking module
  and prints it to the Serial Monitor in fixed-width columns so the text
  lines up and doesn't jump around as the numbers change. A display can
  be driven off the same `latest` data later, in addition to or instead
  of the Serial print.
*/
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>

const int ESPNOW_CHANNEL = 1;              // must match the sender sketch
const uint32_t PRINT_INTERVAL_MS = 100;    // throttle printing to ~10 Hz

// Must exactly match the struct on the sender side.
typedef struct __attribute__((packed)) {
  float heading;
  bool inStance;
  uint32_t stepCount;
  bool footIsLeft;
} FootPacket;

volatile FootPacket latest;
volatile bool haveData = false;
volatile uint32_t packetCount = 0;

// NOTE: this callback signature is for Arduino-ESP32 core 3.x.
// On core 2.x, use instead:
//   void onDataRecv(const uint8_t *mac, const uint8_t *data, int len)
void onDataRecv(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
  if (len != sizeof(FootPacket)) return;
  memcpy((void *)&latest, data, sizeof(FootPacket));
  haveData = true;
  packetCount++;
}

static void initEspNow() {
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  esp_wifi_set_channel(ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE);

  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init failed");
    while (true) delay(1000);
  }
  esp_now_register_recv_cb(onDataRecv);
  Serial.print("This board's MAC: ");
  Serial.println(WiFi.macAddress());
  Serial.println("Waiting for foot module...");
}

void setup() {
  Serial.begin(115200);
  delay(300);
  initEspNow();
}

void loop() {
  static uint32_t lastPrint = 0;
  uint32_t now = millis();
  if (!haveData || now - lastPrint < PRINT_INTERVAL_MS) return;
  lastPrint = now;

  FootPacket pkt;
  noInterrupts();
  pkt = *(FootPacket *)&latest;
  interrupts();

  const char *foot  = pkt.footIsLeft ? "L" : "R";
  const char *state = pkt.inStance ? "STANCE" : "SWING ";   // fixed width
  const char *dir   = pkt.heading >= 0 ? "toe-out" : "toe-in";

  // %+7.1f  -> sign + digits + decimal, fixed 7-char field
  // %-6s    -> STANCE/SWING padded to the same width
  // %-7s    -> toe-out/toe-in padded to the same width
  // %6lu    -> step count right-aligned in 6 chars
  Serial.printf("foot:%s  state:%s  heading:%+7.1f deg (%-7s)  steps:%6lu\n",
                foot, state, pkt.heading, dir, (unsigned long)pkt.stepCount);
}
