#include "IMU_MPU6500.h"
#include "Config.h"
#include <SPI.h>

static SPISettings mpuSettings(1000000, MSBFIRST, SPI_MODE0);

static const uint8_t WHO_AM_I_REG = 0x75;
static const uint8_t PWR_MGMT_1 = 0x6B;
static const uint8_t USER_CTRL = 0x6A;
static const uint8_t GYRO_CONFIG = 0x1B;
static const uint8_t ACCEL_CONFIG = 0x1C;
static const uint8_t ACCEL_XOUT_H = 0x3B;
static const uint8_t MPU6500_WHO_AM_I_VALUE = 0x70;

static float gyroBiasX = 0;
static float gyroBiasY = 0;
static float gyroBiasZ = 0;

static void imuWrite(uint8_t reg, uint8_t val) {
  SPI.beginTransaction(mpuSettings);
  digitalWrite(IMU_CS_PIN, LOW);
  SPI.transfer(reg & 0x7F);
  SPI.transfer(val);
  digitalWrite(IMU_CS_PIN, HIGH);
  SPI.endTransaction();
}

static uint8_t imuReadReg(uint8_t reg) {
  SPI.beginTransaction(mpuSettings);
  digitalWrite(IMU_CS_PIN, LOW);
  SPI.transfer(reg | 0x80);
  uint8_t val = SPI.transfer(0x00);
  digitalWrite(IMU_CS_PIN, HIGH);
  SPI.endTransaction();
  return val;
}

bool imuBegin() {
  pinMode(IMU_CS_PIN, OUTPUT);
  digitalWrite(IMU_CS_PIN, HIGH);

  delay(100);

  uint8_t who = imuReadReg(WHO_AM_I_REG);
  if (who != MPU6500_WHO_AM_I_VALUE) {
    Serial.print("  IMU WHO_AM_I mismatch, got 0x");
    Serial.println(who, HEX);
    return false;
  }

  imuWrite(PWR_MGMT_1, 0x00);
  delay(100);

  imuWrite(USER_CTRL, 0x10);
  delay(10);

  imuWrite(GYRO_CONFIG, 0x18);
  imuWrite(ACCEL_CONFIG, 0x18);

  return true;
}

void imuReadRaw(int16_t &ax, int16_t &ay, int16_t &az,
                int16_t &gx, int16_t &gy, int16_t &gz) {
  uint8_t buf[14];
  SPI.beginTransaction(mpuSettings);
  digitalWrite(IMU_CS_PIN, LOW);
  SPI.transfer(ACCEL_XOUT_H | 0x80);
  for (int i = 0; i < 14; i++) buf[i] = SPI.transfer(0x00);
  digitalWrite(IMU_CS_PIN, HIGH);
  SPI.endTransaction();

  ax = (int16_t)((buf[0] << 8) | buf[1]);
  ay = (int16_t)((buf[2] << 8) | buf[3]);
  az = (int16_t)((buf[4] << 8) | buf[5]);
  gx = (int16_t)((buf[8] << 8) | buf[9]);
  gy = (int16_t)((buf[10] << 8) | buf[11]);
  gz = (int16_t)((buf[12] << 8) | buf[13]);
}

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

void imuGetCorrectedGyro(int16_t gx, int16_t gy, int16_t gz,
                          float &gx_out, float &gy_out, float &gz_out) {
  gx_out = gx - gyroBiasX;
  gy_out = gy - gyroBiasY;
  gz_out = gz - gyroBiasZ;
}

void imuReadG(float &ax_g, float &ay_g, float &az_g,
              float &gx_out, float &gy_out, float &gz_out) {
  int16_t ax, ay, az, gx, gy, gz;
  imuReadRaw(ax, ay, az, gx, gy, gz);

  ax_g = ax / ACCEL_LSB_PER_G;
  ay_g = ay / ACCEL_LSB_PER_G;
  az_g = az / ACCEL_LSB_PER_G;

  imuGetCorrectedGyro(gx, gy, gz, gx_out, gy_out, gz_out);
}

float imuAccelMagnitudeG(float ax_g, float ay_g, float az_g) {
  return sqrt(ax_g * ax_g + ay_g * ay_g + az_g * az_g);
}
