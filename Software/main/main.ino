#include <SPI.h>
#include "Config.h"
#include "IMU_MPU6500.h"
#include "Barometer_BMP280.h"
#include "SDLogger.h"

bool imuOK = false;
bool baroOK = false;
bool sdOK = false;

void setup() {
  Serial.begin(SERIAL_BAUD);
  while (!Serial) delay(10);

  SPI.begin();
  delay(100);

  Serial.println("=== Avionics Bay Self-Test ===");

  imuOK = imuBegin();
  Serial.print("IMU (MPU-6500): ");
  Serial.println(imuOK ? "PASS" : "FAIL");

  baroOK = baroBegin();
  Serial.print("Barometer (BMP280): ");
  Serial.println(baroOK ? "PASS" : "FAIL");

  sdOK = sdBegin();
  Serial.print("SD Card: ");
  Serial.println(sdOK ? "PASS" : "FAIL");

  if (imuOK) {
    imuCalibrate();
  }

  if (baroOK) {
    baroCaptureGroundReference();
  }

  Serial.println("=== Self-Test Complete ===");
}

void loop() {
  String logLine = "";

  if (imuOK) {
    int16_t ax, ay, az, gx, gy, gz;
    float gx_c, gy_c, gz_c;

    imuReadRaw(ax, ay, az, gx, gy, gz);
    imuGetCorrectedGyro(gx, gy, gz, gx_c, gy_c, gz_c);

    Serial.print("Accel X: "); Serial.print(ax);
    Serial.print(" Y: "); Serial.print(ay);
    Serial.print(" Z: "); Serial.println(az);

    Serial.print("Gyro X: "); Serial.print(gx_c);
    Serial.print(" Y: "); Serial.print(gy_c);
    Serial.print(" Z: "); Serial.println(gz_c);

    logLine += "A:" + String(ax) + "," + String(ay) + "," + String(az);
    logLine += " G:" + String(gx_c) + "," + String(gy_c) + "," + String(gz_c);
  }

  if (baroOK) {
    float altitude = baroReadAltitude();
    Serial.print("Altitude: ");
    Serial.print(altitude);
    Serial.println(" m");

    logLine += " Alt:" + String(altitude);
  }

  if (sdOK && logLine.length() > 0) {
    sdWriteLine(logLine);
  }

  delay(1000);
}
