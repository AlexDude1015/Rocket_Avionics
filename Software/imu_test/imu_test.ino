#include <SPI.h>

const int CS_PIN = 3;   // pick any free digital pin, match your schematic

SPISettings mpu_settings(1000000, MSBFIRST, SPI_MODE0); // 1 MHz for register access

// --- Low-level SPI helpers ---
void spi_write(uint8_t reg, uint8_t val) {
  SPI.beginTransaction(mpu_settings);
  digitalWrite(CS_PIN, LOW);
  SPI.transfer(reg & 0x7F);   // MSB = 0 for write
  SPI.transfer(val);
  digitalWrite(CS_PIN, HIGH);
  SPI.endTransaction();
}

uint8_t spi_read(uint8_t reg) {
  SPI.beginTransaction(mpu_settings);
  digitalWrite(CS_PIN, LOW);
  SPI.transfer(reg | 0x80);   // MSB = 1 for read
  uint8_t val = SPI.transfer(0x00);
  digitalWrite(CS_PIN, HIGH);
  SPI.endTransaction();
  return val;
}

// --- Register addresses (MPU-6500) ---
#define WHO_AM_I      0x75
#define PWR_MGMT_1    0x6B
#define GYRO_CONFIG   0x1B
#define ACCEL_CONFIG  0x1C
#define ACCEL_XOUT_H  0x3B   // start of 14-byte burst: accel(6) + temp(2) + gyro(6)

// --- Gyro bias values, filled in during calibration ---
float gyro_bias_x = 0;
float gyro_bias_y = 0;
float gyro_bias_z = 0;

void readRawData(int16_t &ax, int16_t &ay, int16_t &az,
                  int16_t &gx, int16_t &gy, int16_t &gz) {
  uint8_t buf[14];
  SPI.beginTransaction(mpu_settings);
  digitalWrite(CS_PIN, LOW);
  SPI.transfer(ACCEL_XOUT_H | 0x80);
  for (int i = 0; i < 14; i++) buf[i] = SPI.transfer(0x00);
  digitalWrite(CS_PIN, HIGH);
  SPI.endTransaction();

  ax = (int16_t)((buf[0] << 8) | buf[1]);
  ay = (int16_t)((buf[2] << 8) | buf[3]);
  az = (int16_t)((buf[4] << 8) | buf[5]);
  // buf[6],buf[7] = temperature — skip for now
  gx = (int16_t)((buf[8] << 8) | buf[9]);
  gy = (int16_t)((buf[10] << 8) | buf[11]);
  gz = (int16_t)((buf[12] << 8) | buf[13]);
}

void calibrateGyro() {
  const int N = 500;
  int16_t ax, ay, az, gx, gy, gz;

  Serial.println("Calibrating gyro — keep the sensor still...");

  for (int i = 0; i < N; i++) {
    readRawData(ax, ay, az, gx, gy, gz);
    gyro_bias_x += gx;
    gyro_bias_y += gy;
    gyro_bias_z += gz;
    delay(2);
  }

  gyro_bias_x /= N;
  gyro_bias_y /= N;
  gyro_bias_z /= N;

  Serial.println("Calibration done.");
  Serial.print("Bias X: "); Serial.print(gyro_bias_x);
  Serial.print(" Y: "); Serial.print(gyro_bias_y);
  Serial.print(" Z: "); Serial.println(gyro_bias_z);
}

void setup() {
  Serial.begin(9600);
  while (!Serial) delay(10);

  pinMode(CS_PIN, OUTPUT);
  digitalWrite(CS_PIN, HIGH);
  SPI.begin();

  delay(100); // let power rails settle

  uint8_t who = spi_read(WHO_AM_I);
  Serial.print("WHO_AM_I: 0x");
  Serial.println(who, HEX);

  spi_write(PWR_MGMT_1, 0x00);   // wake from sleep, use internal clock
  delay(100);

  spi_write(GYRO_CONFIG, 0x18);   // ±2000 dps
  spi_write(ACCEL_CONFIG, 0x18);  // ±16g

  calibrateGyro();
}

void loop() {
  int16_t ax, ay, az, gx, gy, gz;
  readRawData(ax, ay, az, gx, gy, gz);

  float gx_corrected = gx - gyro_bias_x;
  float gy_corrected = gy - gyro_bias_y;
  float gz_corrected = gz - gyro_bias_z;

  Serial.print("Accel X: "); Serial.print(ax);
  Serial.print(" Y: "); Serial.print(ay);
  Serial.print(" Z: "); Serial.println(az);

  Serial.print("Gyro (bias-corrected) X: "); Serial.print(gx_corrected);
  Serial.print(" Y: "); Serial.print(gy_corrected);
  Serial.print(" Z: "); Serial.println(gz_corrected);

  delay(1000);
}