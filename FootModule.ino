/*
  Foot Tracking Module - ESP32 + MPU-9250/6500 (I2C)
  Flash this to the board that actually sits on the shoe.

  Same angle-tracking core as the single-unit prototype:
   - integrates gyro around the gravity direction to get foot yaw
   - freezes yaw + relearns gyro bias while the foot is planted (stance)
   - counts steps and reports toe-out(+)/toe-in(-) angle per step

  Changes from the single-unit version:
   - buzzer feedback is ON HOLD: the startBuzz() call sites are left in
     as commented-out lines so it's a one-line change to bring back
   - every IMU sample is packaged up and sent over ESP-NOW to the
     receiver board, so it can show heading + stance state live
*/
#include <Wire.h>
#include <math.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>

// ============================ USER CONFIG ============================
const bool FOOT_IS_LEFT = true;          // false for the right shoe

// --- Pins: edit to match your board ---
const int PIN_SDA = 21;
const int PIN_SCL = 22;
const int PIN_MOTOR = 25;                // wired but unused while buzzer is on hold
const int PIN_BUTTON = -1;               // set to -1 if you have no button
const int MPU_ADDR = 0x68;               // 0x69 if AD0 is tied high

// --- Receiver's MAC address (the plotting board) ---
uint8_t RECEIVER_MAC[6] = { 0xF4, 0x2D, 0xC9, 0x6A, 0x9B, 0xAC };
const int ESPNOW_CHANNEL = 1;            // must match the receiver sketch

// --- Feedback limits (kept for later; buzzer calls below are disabled) ---
const float TOE_OUT_LIMIT_DEG = 10.0f;
const float TOE_IN_LIMIT_DEG = 10.0f;

// --- Gait detection tuning ---
const float STANCE_GYRO_DPS = 30.0f;
const float STANCE_ACCEL_G = 0.12f;
const int STANCE_CONFIRM = 5;
const float SWING_GYRO_DPS = 80.0f;
const float BIAS_UPDATE_DPS = 10.0f;
const float BIAS_ALPHA = 0.02f;
const float TILT_GAIN_STANCE = 0.10f;
const float TILT_GAIN_MOVING = 0.005f;
// =====================================================================

const uint32_t SAMPLE_US = 10000;        // 100 Hz
const int CAL_SAMPLES = 200;             // 2 s of stillness at startup/recalibration
const float D2R = 0.017453292f;
const float R2D = 57.29578f;

struct Vec3 { float x, y, z; };
static inline Vec3 vadd(Vec3 a, Vec3 b) { return { a.x + b.x, a.y + b.y, a.z + b.z }; }
static inline Vec3 vsub(Vec3 a, Vec3 b) { return { a.x - b.x, a.y - b.y, a.z - b.z }; }
static inline Vec3 vscale(Vec3 a, float s){ return { a.x * s, a.y * s, a.z * s }; }
static inline float vdot(Vec3 a, Vec3 b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
static inline float vlen(Vec3 a) { return sqrtf(vdot(a, a)); }
static inline Vec3 vcross(Vec3 a, Vec3 b) {
  return { a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x };
}
static inline Vec3 vnorm(Vec3 a) {
  float m = vlen(a);
  if (m < 1e-6f) return { 0.0f, 0.0f, 1.0f };
  return vscale(a, 1.0f / m);
}

// ---------------- Packet sent to the receiver ----------------
typedef struct __attribute__((packed)) {
  float heading;        // toe-out angle, deg (positive = toe-out)
  bool inStance;        // true while the foot is planted
  uint32_t stepCount;   // total steps since last calibration
  bool footIsLeft;      // which foot this packet is from
} FootPacket;

FootPacket outPacket;

// ---------------- State ----------------
Vec3 gyroBias = { 0, 0, 0 };
Vec3 upBody = { 0, 0, 1 };
float heading = 0.0f;
int stanceCount = 0;
bool sawSwing = false;
uint32_t stepCount = 0;
uint32_t lastUs = 0;

// ---------------- IMU (unchanged register-level driver, no MPU library) ----------------
static void writeReg(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
}
static bool readRegs(uint8_t reg, uint8_t *buf, int len) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  if ((int)Wire.requestFrom(MPU_ADDR, len, 1) != len) return false;
  for (int i = 0; i < len; i++) buf[i] = Wire.read();
  return true;
}
static bool initIMU() {
  uint8_t id = 0;
  if (!readRegs(0x75, &id, 1)) return false;
  Serial.printf("WHO_AM_I = 0x%02X\n", id);
  writeReg(0x6B, 0x80); delay(100); // reset
  writeReg(0x6B, 0x01); delay(50);  // wake, PLL clock
  writeReg(0x1A, 0x03);             // gyro low-pass ~41 Hz
  writeReg(0x19, 0x09);             // 1 kHz / (1+9) = 100 Hz
  writeReg(0x1B, 0x10);             // gyro range +/-1000 dps
  writeReg(0x1C, 0x10);             // accel range +/-8 g
  writeReg(0x1D, 0x03);             // accel low-pass ~44 Hz
  delay(50);
  return true;
}
static bool readIMU(Vec3 &a, Vec3 &w) {
  uint8_t b[14];
  if (!readRegs(0x3B, b, 14)) return false;
  int16_t rax = (int16_t)((b[0] << 8) | b[1]);
  int16_t ray = (int16_t)((b[2] << 8) | b[3]);
  int16_t raz = (int16_t)((b[4] << 8) | b[5]);
  int16_t rgx = (int16_t)((b[8] << 8) | b[9]);
  int16_t rgy = (int16_t)((b[10] << 8) | b[11]);
  int16_t rgz = (int16_t)((b[12] << 8) | b[13]);
  a = { rax / 4096.0f, ray / 4096.0f, raz / 4096.0f }; // +/-8 g
  w = { rgx / 32.8f * D2R, rgy / 32.8f * D2R, rgz / 32.8f * D2R }; // +/-1000 dps
  return true;
}

static void calibrate() {
  Serial.println("Calibrating: keep the shoe still, foot pointing straight...");
  Vec3 sumW = { 0, 0, 0 }, sumA = { 0, 0, 0 };
  int n = 0;
  uint32_t t = micros();
  while (n < CAL_SAMPLES) {
    if (micros() - t >= SAMPLE_US) {
      t += SAMPLE_US;
      Vec3 a, w;
      if (readIMU(a, w)) {
        sumW = vadd(sumW, w);
        sumA = vadd(sumA, a);
        n++;
      }
    }
  }
  gyroBias = vscale(sumW, 1.0f / n);
  upBody = vnorm(sumA);
  heading = 0.0f;
  stanceCount = 0;
  sawSwing = false;
  stepCount = 0;
  lastUs = micros();
  // startBuzz(1, 80, 0);   // buzzer on hold for now
  Serial.println("Calibrated. Walk!");
}

static void handleInputs() {
  static uint32_t lastPress = 0;
  bool pressed = (PIN_BUTTON >= 0) && (digitalRead(PIN_BUTTON) == LOW);
  bool serialCmd = false;
  while (Serial.available()) {
    if (Serial.read() == 'c') serialCmd = true;
  }
  if ((pressed && millis() - lastPress > 500) || serialCmd) {
    calibrate();
    lastPress = millis();
  }
}

// ---------------- ESP-NOW ----------------
void onDataSent(const uint8_t *mac, esp_now_send_status_t status) {
  // Uncomment to debug dropped packets:
  // Serial.println(status == ESP_NOW_SEND_SUCCESS ? "sent ok" : "send failed");
}

static void initEspNow() {
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  esp_wifi_set_channel(ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE);

  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init failed");
    while (true) delay(1000);
  }
  esp_now_register_send_cb(onDataSent);

  esp_now_peer_info_t peer = {};
  memcpy(peer.peer_addr, RECEIVER_MAC, 6);
  peer.channel = ESPNOW_CHANNEL;
  peer.encrypt = false;
  if (esp_now_add_peer(&peer) != ESP_OK) {
    Serial.println("Failed to add receiver as ESP-NOW peer");
  }
  Serial.print("This board's MAC: ");
  Serial.println(WiFi.macAddress());
}

void setup() {
  Serial.begin(115200);
  delay(300);
  if (PIN_BUTTON >= 0) pinMode(PIN_BUTTON, INPUT_PULLUP);
  pinMode(PIN_MOTOR, OUTPUT);
  digitalWrite(PIN_MOTOR, LOW);

  Wire.begin(PIN_SDA, PIN_SCL);
  Wire.setClock(400000);
  if (!initIMU()) {
    Serial.println("IMU not responding. Check wiring, power, and AD0/address.");
    while (true) delay(1000);
  }
  initEspNow();
  calibrate();
}

void loop() {
  handleInputs();

  uint32_t nowUs = micros();
  if (nowUs - lastUs < SAMPLE_US) return;
  float dt = (nowUs - lastUs) * 1e-6f;
  lastUs = nowUs;

  Vec3 a, wRaw;
  if (!readIMU(a, wRaw)) return;

  Vec3 w = vsub(wRaw, gyroBias);
  float wDps = vlen(w) * R2D;
  float aMag = vlen(a);

  bool still = (wDps < STANCE_GYRO_DPS) && (fabsf(aMag - 1.0f) < STANCE_ACCEL_G);
  if (still) { if (stanceCount < 1000) stanceCount++; }
  else stanceCount = 0;
  bool inStance = (stanceCount >= STANCE_CONFIRM);

  if (inStance && wDps < BIAS_UPDATE_DPS) {
    gyroBias = vadd(gyroBias, vscale(vsub(wRaw, gyroBias), BIAS_ALPHA));
  }

  Vec3 upPred = vsub(upBody, vscale(vcross(w, upBody), dt));
  if (fabsf(aMag - 1.0f) < 0.15f) {
    float k = inStance ? TILT_GAIN_STANCE : TILT_GAIN_MOVING;
    upBody = vnorm(vadd(vscale(upPred, 1.0f - k), vscale(vnorm(a), k)));
  } else {
    upBody = vnorm(upPred);
  }

  if (!inStance) {
    heading += vdot(w, upBody) * R2D * dt;
  }

  if (wDps > SWING_GYRO_DPS) sawSwing = true;

  if (stanceCount == STANCE_CONFIRM && sawSwing) {
    sawSwing = false;
    stepCount++;
    float toeOutStep = FOOT_IS_LEFT ? heading : -heading;
    Serial.printf("Step %lu: %+.1f deg (%s)\n",
                  (unsigned long)stepCount, toeOutStep,
                  toeOutStep >= 0 ? "toe-out" : "toe-in");
    // if (toeOutStep > TOE_OUT_LIMIT_DEG) startBuzz(1, 300, 0);        // buzzer on hold
    // else if (toeOutStep < -TOE_IN_LIMIT_DEG) startBuzz(2, 100, 100); // buzzer on hold
  }

  // --- Send the live state to the receiver every sample (100 Hz) ---
  float toeOut = FOOT_IS_LEFT ? heading : -heading;
  outPacket.heading = toeOut;
  outPacket.inStance = inStance;
  outPacket.stepCount = stepCount;
  outPacket.footIsLeft = FOOT_IS_LEFT;
  esp_now_send(RECEIVER_MAC, (uint8_t *)&outPacket, sizeof(outPacket));
}
