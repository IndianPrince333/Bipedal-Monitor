/*
  Wrist receiver ("ESP-OLED-Motor")
  ESP32 + small I2C OLED (SSD1306 128x64) + offset-weight vibration motor + recalibrate button

  Wiring:
    GND    -> GND
    3.3V   -> Vin
    GPIO21 -> SDA
    GPIO22 -> SCL
    GPIO25 -> Motor+ (driven full power, not PWM -- offset weight motor)
    GPIO4  -> Recalibrate button (other side of button to GND, uses internal pull-up)

  NOTE: time shown is a placeholder clock (no RTC/NTP wired up yet) --
  starts at 14:30:00 and just counts up. Swap in an RTC or NTP sync later.

  Requires: Adafruit_GFX and Adafruit_SSD1306 libraries (Library Manager).
*/

#include <WiFi.h>
#include <esp_now.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// ---------------- Pins ----------------
#define SDA_PIN 21
#define SCL_PIN 22
#define MOTOR_PIN 25
#define BUTTON_PIN 4

// ---------------- OLED ----------------
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_ADDR 0x3C
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

// ---------------- ESP-NOW ----------------
// Foot transmitter MAC
uint8_t transmitterMac[] = {0x14, 0x08, 0x08, 0xA4, 0xEC, 0x24};

// Packet definitions -- MUST match the transmitter sketch exactly
typedef struct {
  float yawDeg;
  uint8_t status;   // 0 = walking correct, 1 = walking wrong
  uint32_t seq;
} FootPacket;

typedef struct {
  uint8_t cmd;      // 1 = recalibrate
} CalibCommand;

volatile FootPacket lastPacket = {0, 0, 0};
volatile unsigned long lastPacketMs = 0;

const unsigned long SIGNAL_TIMEOUT_MS = 2000;   // no packet in this long -> "NO SIGNAL"
const unsigned long MOTOR_MIN_STATE_MS = 3000;  // minimum hold time before flipping motor state

bool motorOn = false;
unsigned long lastMotorChangeMs = 0;

// ---------------- Placeholder clock ----------------
int clockH = 14, clockM = 30, clockS = 0;
unsigned long lastClockTickMs = 0;

// ---------------- Button debounce ----------------
bool lastButtonState = HIGH;
unsigned long lastButtonChangeMs = 0;
const unsigned long DEBOUNCE_MS = 50;

// ---------------- ESP-NOW receive ----------------
void onDataRecv(const uint8_t *mac, const uint8_t *data, int len) {
  if (len == sizeof(FootPacket)) {
    memcpy((void*)&lastPacket, data, sizeof(FootPacket));
    lastPacketMs = millis();
  }
}

void sendCalibrate() {
  CalibCommand cc;
  cc.cmd = 1;
  esp_now_send(transmitterMac, (uint8_t*)&cc, sizeof(cc));
}

// ---------------- Clock ----------------
void updateClock() {
  if (millis() - lastClockTickMs >= 1000) {
    lastClockTickMs += 1000;
    clockS++;
    if (clockS >= 60) { clockS = 0; clockM++; }
    if (clockM >= 60) { clockM = 0; clockH++; }
    if (clockH >= 24) clockH = 0;
  }
}

// ---------------- Motor (with hysteresis) ----------------
void updateMotor(bool signalOk) {
  bool wantOn = signalOk && (lastPacket.status == 1);
  unsigned long sinceChange = millis() - lastMotorChangeMs;
  if (wantOn != motorOn && sinceChange >= MOTOR_MIN_STATE_MS) {
    motorOn = wantOn;
    lastMotorChangeMs = millis();
    digitalWrite(MOTOR_PIN, motorOn ? HIGH : LOW);
  }
}

// ---------------- Button ----------------
void checkButton() {
  bool reading = digitalRead(BUTTON_PIN);
  if (reading != lastButtonState) {
    lastButtonChangeMs = millis();
  }
  if (millis() - lastButtonChangeMs > DEBOUNCE_MS) {
    if (reading == LOW && lastButtonState == HIGH) {
      Serial.println("Recalibrate button pressed -> sending command");
      sendCalibrate();
    }
  }
  lastButtonState = reading;
}

// ---------------- Display ----------------
void drawScreen(bool signalOk) {
  display.clearDisplay();

  // Big time, centered
  display.setTextSize(3);
  display.setTextColor(SSD1306_WHITE);
  char buf[9];
  snprintf(buf, sizeof(buf), "%02d:%02d:%02d", clockH, clockM, clockS);
  int16_t x1, y1; uint16_t w, h;
  display.getTextBounds(buf, 0, 0, &x1, &y1, &w, &h);
  display.setCursor((SCREEN_WIDTH - w) / 2, (SCREEN_HEIGHT - h) / 2 - 6);
  display.print(buf);

  // Small status line at the bottom
  display.setTextSize(1);
  const char* statusText;
  if (!signalOk) statusText = "NO SIGNAL";
  else statusText = (lastPacket.status == 1) ? "FIX FOOT ANGLE" : "WALKING OK";
  display.getTextBounds(statusText, 0, 0, &x1, &y1, &w, &h);
  display.setCursor((SCREEN_WIDTH - w) / 2, SCREEN_HEIGHT - 10);
  display.print(statusText);

  display.display();
}

void setup() {
  Serial.begin(115200);

  pinMode(MOTOR_PIN, OUTPUT);
  digitalWrite(MOTOR_PIN, LOW);
  pinMode(BUTTON_PIN, INPUT_PULLUP);

  Wire.begin(SDA_PIN, SCL_PIN);
  if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)) {
    Serial.println("OLED init failed -- check wiring/address");
  }
  display.clearDisplay();
  display.display();

  WiFi.mode(WIFI_STA);
  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init failed");
    return;
  }
  esp_now_register_recv_cb(onDataRecv);

  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, transmitterMac, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;
  esp_now_add_peer(&peerInfo);

  lastClockTickMs = millis();
}

void loop() {
  updateClock();
  checkButton();

  bool signalOk = (millis() - lastPacketMs) < SIGNAL_TIMEOUT_MS;
  updateMotor(signalOk);
  drawScreen(signalOk);

  delay(50);
}
