#include "Barometer_BMP280.h"
#include "Config.h"
#include <SPI.h>
#include <Adafruit_BMP280.h>

static Adafruit_BMP280 bmp(BARO_CS_PIN);
static float pGround = 0;
static bool frozen = false;

// Called at 1 Hz, so this is a ~8 minute time constant: fast enough to track
// weather drift over a long pad hold, far too slow to chase a real ascent.
static const float REBASELINE_ALPHA = 0.002f;

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
  frozen = false;

  Serial.print("  Ground pressure: ");
  Serial.println(pGround);
}

void baroRebaselineStep() {
  if (frozen) return;
  float p = bmp.readPressure() / 100.0;
  pGround = pGround * (1.0f - REBASELINE_ALPHA) + p * REBASELINE_ALPHA;
}

void baroFreezeGroundReference() {
  frozen = true;
}

bool baroGroundReferenceFrozen() {
  return frozen;
}

float baroGroundPressure() {
  return pGround;
}

float baroReadAltitude() {
  return bmp.readAltitude(pGround);
}

float baroReadPressure() {
  return bmp.readPressure() / 100.0;
}
