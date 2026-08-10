#include "Barometer_BMP280.h"
#include "Config.h"
#include <SPI.h>
#include <Adafruit_BMP280.h>

static Adafruit_BMP280 bmp(BARO_CS_PIN);
static float pGround = 0;

bool baroBegin() {
  return bmp.begin();
}

void baroCaptureGroundReference() {
  pGround = 0;

  for (int i = 0; i < 20; i++) {
    pGround += bmp.readPressure() / 100.0;
    delay(100);
  }
  pGround /= 20;

  Serial.print("  Ground pressure: ");
  Serial.println(pGround);
}

float baroReadAltitude() {
  return bmp.readAltitude(pGround);
}

float baroReadPressure() {
  return bmp.readPressure() / 100.0;
}
