/*
  Foot-angle transmitter (shoe unit)
  ESP32 + MPU9250 (register-level I2C driver, no library)
  Sends yaw + walking-status packets to the wrist receiver over ESP-NOW.

  Wiring:
    GND  -> GND
    3.3V -> VCC
    GPIO22 -> SCL
    GPIO21 -> SDA
    AD0  -> GND   (I2C address 0x68)
    NCS  -> 3.3V  (forces I2C mode on GY-91 style breakouts)
*/

#include <WiFi.h>
#include <esp_now.h>
#include <Wire.h>

// ---------------- Pins ----------------
#define SDA_PIN 21
#define SCL_PIN 22

// ---------------- MPU9250 registers ----------------
#define MPU_ADDR        0x68
#define REG_PWR_MGMT_1  0x6B
#define REG_GYRO_CONFIG 0x1B
#define REG_CONFIG      0x1A
#define REG_SMPLRT_DIV  0x19
#define REG_GYRO_ZOUT_H 0x47
#define REG_WHO_AM_I    0x75

#define GYRO_SENS_500DPS 65.5f  // LSB per (deg/s) at +/-500dps full scale

// ---------------- ESP-NOW ----------------
// Receiver / wrist unit MAC
uint8_t receiverMac[] = {0xF4, 0x2D, 0xC9, 0x6A, 0x9B, 0xAC};

// Packet definitions -- MUST match the receiver sketch exactly
typedef struct {
  float yawDeg;
  uint8_t status;   // 0 = walking correct, 1 = walking wrong (past threshold either direction)
  uint32_t seq;
} FootPacket;

typedef struct {
  uint8_t cmd;      // 1 = recalibrate
} CalibCommand;

FootPacket packet;
volatile bool calibrateRequested = false;

// ---------------- State ----------------
float yawDeg = 0.0f;
float gyroZBias = 0.0f;
unsigned long lastMicros = 0;
uint32_t seqCounter = 0;

const float WRONG_THRESHOLD_DEG = 7.5f;      // +/- degrees before flagging "wrong"
const unsigned long SEND_INTERVAL_MS = 20;   // 50 Hz
unsigned long lastSendMs = 0;

// ---------------- Low-level I2C helpers ----------------
void writeReg(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
}

void readRegs(uint8_t reg, uint8_t count, uint8_t *buf) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom((int)MPU_ADDR, (int)count);
  for (uint8_t i = 0; i < count && Wire.available(); i++) {
    buf[i] = Wire.read();
  }
}

int16_t readGyroZRaw() {
  uint8_t buf[2];
  readRegs(REG_GYRO_ZOUT_H, 2, buf);
  return (int16_t)((buf[0] << 8) | buf[1]);
}

// ---------------- Calibration ----------------
// Zeroes gyro bias AND resets yaw to 0. Foot should be held still/straight
// while this runs -- called at boot and again on recalibrate command.
void calibrateGyro() {
  const int N = 200;
  long sum = 0;
  for (int i = 0; i < N; i++) {
    sum += readGyroZRaw();
    delay(3);
  }
  gyroZBias = (float)sum / N;
  yawDeg = 0.0f;
  lastMicros = micros();
}

// ---------------- ESP-NOW callback ----------------
void onDataRecv(const uint8_t *mac, const uint8_t *data, int len) {
  if (len == sizeof(CalibCommand)) {
    CalibCommand cc;
    memcpy(&cc, data, sizeof(cc));
    if (cc.cmd == 1) {
      calibrateRequested = true;
    }
  }
}

void setup() {
  Serial.begin(115200);
  Wire.begin(SDA_PIN, SCL_PIN);
  delay(100);

  writeReg(REG_PWR_MGMT_1, 0x00);   // wake device up
  delay(100);
  writeReg(REG_GYRO_CONFIG, 0x08);  // +/-500 dps
  writeReg(REG_CONFIG, 0x03);       // DLPF ~44Hz bandwidth
  writeReg(REG_SMPLRT_DIV, 0x04);   // internal sample rate ~200Hz
  delay(50);

  Serial.println("Hold foot still/straight for startup calibration...");
  calibrateGyro();
  Serial.println("Calibrated.");

  WiFi.mode(WIFI_STA);
  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init failed");
    return;
  }
  esp_now_register_recv_cb(onDataRecv);

  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, receiverMac, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;
  esp_now_add_peer(&peerInfo);

  lastMicros = micros();
}

void loop() {
  if (calibrateRequested) {
    Serial.println("Recalibrating (hold foot still)...");
    calibrateGyro();
    calibrateRequested = false;
  }

  unsigned long now = micros();
  float dt = (now - lastMicros) / 1000000.0f;
  lastMicros = now;

  int16_t rawZ = readGyroZRaw();
  float dps = (rawZ - gyroZBias) / GYRO_SENS_500DPS;
  yawDeg += dps * dt;

  if (millis() - lastSendMs >= SEND_INTERVAL_MS) {
    lastSendMs = millis();
    packet.yawDeg = yawDeg;
    packet.status = (fabs(yawDeg) > WRONG_THRESHOLD_DEG) ? 1 : 0;
    packet.seq = seqCounter++;
    esp_now_send(receiverMac, (uint8_t*)&packet, sizeof(packet));
  }
}
