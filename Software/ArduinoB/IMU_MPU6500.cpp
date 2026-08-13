#include "IMU_MPU6500.h"
#include "Config.h"
#include <SPI.h>

// 1 MHz is well inside the MPU-6500's 1 MHz register-access limit. Mode 0 and
// MSB-first are what the part expects.
static SPISettings mpuSettings(1000000, MSBFIRST, SPI_MODE0);

// Register addresses from the MPU-6500 datasheet.
static const uint8_t WHO_AM_I_REG = 0x75;   // identity, should read 0x70
static const uint8_t PWR_MGMT_1 = 0x6B;     // sleep / clock source
static const uint8_t USER_CTRL = 0x6A;      // interface control
static const uint8_t GYRO_CONFIG = 0x1B;    // gyro full-scale range
static const uint8_t ACCEL_CONFIG = 0x1C;   // accel full-scale range
static const uint8_t ACCEL_XOUT_H = 0x3B;   // first of 14 sequential data bytes
static const uint8_t MPU6500_WHO_AM_I_VALUE = 0x70;

// Gyroscope zero-rate offset, filled in by imuCalibrate(). Every gyro reads
// some non-zero rate while perfectly still; this captures that so it can be
// subtracted back out.
static float gyroBiasX = 0;
static float gyroBiasY = 0;
static float gyroBiasZ = 0;

// Writes one byte to one register.
// The MPU-6500 uses the top bit of the address to pick direction, so clearing
// it with & 0x7F marks this transfer as a write.
//   reg - register address
//   val - byte to store there
static void imuWrite(uint8_t reg, uint8_t val) {
  SPI.beginTransaction(mpuSettings);
  digitalWrite(IMU_CS_PIN, LOW);      // chip select is active low
  SPI.transfer(reg & 0x7F);
  SPI.transfer(val);
  digitalWrite(IMU_CS_PIN, HIGH);
  SPI.endTransaction();
}

// Reads one byte from one register.
// Setting the top bit with | 0x80 marks the transfer as a read; the dummy
// 0x00 byte then clocks the sensor's answer back out.
//   reg - register address
// Returns the byte held in that register.
static uint8_t imuReadReg(uint8_t reg) {
  SPI.beginTransaction(mpuSettings);
  digitalWrite(IMU_CS_PIN, LOW);
  SPI.transfer(reg | 0x80);
  uint8_t val = SPI.transfer(0x00);
  digitalWrite(IMU_CS_PIN, HIGH);
  SPI.endTransaction();
  return val;
}

// See IMU_MPU6500.h. Identity is checked before configuring anything, so a
// wiring fault is reported plainly instead of silently producing zeroes.
bool imuBegin() {
  pinMode(IMU_CS_PIN, OUTPUT);
  digitalWrite(IMU_CS_PIN, HIGH);     // deselect until we actually talk to it

  delay(100);                         // let the power rails settle

  uint8_t who = imuReadReg(WHO_AM_I_REG);
  if (who != MPU6500_WHO_AM_I_VALUE) {
    // 0x00 here almost always means nothing is answering on the bus at all.
    Serial.print("  IMU WHO_AM_I mismatch, got 0x");
    Serial.println(who, HEX);
    return false;
  }

  imuWrite(PWR_MGMT_1, 0x00);         // wake from sleep, internal clock
  delay(100);

  imuWrite(USER_CTRL, 0x10);          // disable I2C, force SPI-only
  delay(10);

  imuWrite(GYRO_CONFIG, 0x18);        // +/-2000 dps
  imuWrite(ACCEL_CONFIG, 0x18);       // +/-16g, needed for ~11g boost
  return true;
}

// See IMU_MPU6500.h.
// The 14 bytes are accel XYZ, temperature, then gyro XYZ, each as a big-endian
// signed 16-bit pair. Temperature (buf[6..7]) is skipped. Reading them in one
// burst matters: separate reads would sample the axes at different instants.
void imuReadRaw(int16_t &ax, int16_t &ay, int16_t &az,
                int16_t &gx, int16_t &gy, int16_t &gz) {
  uint8_t buf[14];
  SPI.beginTransaction(mpuSettings);
  digitalWrite(IMU_CS_PIN, LOW);
  SPI.transfer(ACCEL_XOUT_H | 0x80);              // read, auto-incrementing
  for (int i = 0; i < 14; i++) buf[i] = SPI.transfer(0x00);
  digitalWrite(IMU_CS_PIN, HIGH);
  SPI.endTransaction();

  // High byte first, then low byte, cast to signed.
  ax = (int16_t)((buf[0] << 8) | buf[1]);
  ay = (int16_t)((buf[2] << 8) | buf[3]);
  az = (int16_t)((buf[4] << 8) | buf[5]);
  gx = (int16_t)((buf[8] << 8) | buf[9]);
  gy = (int16_t)((buf[10] << 8) | buf[11]);
  gz = (int16_t)((buf[12] << 8) | buf[13]);
}

// See IMU_MPU6500.h.
// Averaging 500 samples suppresses random noise, leaving the constant offset.
// The 2 ms spacing keeps the samples independent rather than re-reading the
// same internal conversion repeatedly.
void imuCalibrate() {
  const int N = 500;
  int16_t ax, ay, az, gx, gy, gz;

  Serial.println("  Calibrating gyro, keep the sensor still...");

  gyroBiasX = 0;
  gyroBiasY = 0;
  gyroBiasZ = 0;

  for (int i = 0; i < N; i++) {
    imuReadRaw(ax, ay, az, gx, gy, gz);
    gyroBiasX += gx;
    gyroBiasY += gy;
    gyroBiasZ += gz;
    delay(2);
  }

  gyroBiasX /= N;
  gyroBiasY /= N;
  gyroBiasZ /= N;

  Serial.print("  Gyro bias X: "); Serial.print(gyroBiasX);
  Serial.print(" Y: "); Serial.print(gyroBiasY);
  Serial.print(" Z: "); Serial.println(gyroBiasZ);
}

// See IMU_MPU6500.h.
void imuGetCorrectedGyro(int16_t gx, int16_t gy, int16_t gz,
                          float &gx_out, float &gy_out, float &gz_out) {
  gx_out = gx - gyroBiasX;
  gy_out = gy - gyroBiasY;
  gz_out = gz - gyroBiasZ;
}

// See IMU_MPU6500.h.
// ACCEL_LSB_PER_G is 2048 because ACCEL_CONFIG is set to +/-16g: the 16-bit
// signed range spans +/-32768 counts over +/-16g.
void imuReadG(float &ax_g, float &ay_g, float &az_g,
              float &gx_out, float &gy_out, float &gz_out) {
  int16_t ax, ay, az, gx, gy, gz;
  imuReadRaw(ax, ay, az, gx, gy, gz);

  ax_g = ax / ACCEL_LSB_PER_G;
  ay_g = ay / ACCEL_LSB_PER_G;
  az_g = az / ACCEL_LSB_PER_G;

  imuGetCorrectedGyro(gx, gy, gz, gx_out, gy_out, gz_out);
}

// See IMU_MPU6500.h.
float imuAccelMagnitudeG(float ax_g, float ay_g, float az_g) {
  return sqrt(ax_g * ax_g + ay_g * ay_g + az_g * az_g);
}
