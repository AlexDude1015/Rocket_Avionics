#include "Barometer_BMP280.h"
#include "Config.h"
#include <SPI.h>
#include <Adafruit_BMP280.h>

// Constructing with a pin selects hardware SPI on that chip-select line.
static Adafruit_BMP280 bmp(BARO_CS_PIN);

// Pressure at ground level, in hPa. Every reported altitude is relative to this.
static float pGround = 0;

// Once true, pGround stops tracking - set at liftoff.
static bool frozen = false;

// Weight given to each new sample when re-baselining. Called at 1 Hz, this
// works out to roughly an 8 minute time constant: quick enough to follow
// weather drift over a long pad hold, far too slow to be fooled by an actual
// ascent even if liftoff detection somehow missed it.
static const float REBASELINE_ALPHA = 0.002f;

// See Barometer_BMP280.h.
bool baroBegin() {
  return bmp.begin();
}

// See Barometer_BMP280.h.
// The driver reports pressure in pascals; everything here works in hPa, hence
// the division by 100. Averaging 20 samples 100 ms apart both smooths noise
// and spans enough time to average out short-term fluctuations.
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

// See Barometer_BMP280.h.
// This is an exponential moving average: the reference keeps almost all of its
// previous value and absorbs a sliver of the new reading, so it creeps rather
// than jumps.
void baroRebaselineStep() {
  if (frozen) return;
  float p = bmp.readPressure() / 100.0;
  pGround = pGround * (1.0f - REBASELINE_ALPHA) + p * REBASELINE_ALPHA;
}

// See Barometer_BMP280.h.
void baroFreezeGroundReference() {
  frozen = true;
}

// See Barometer_BMP280.h.
bool baroGroundReferenceFrozen() {
  return frozen;
}

// See Barometer_BMP280.h.
float baroGroundPressure() {
  return pGround;
}

// See Barometer_BMP280.h.
// readAltitude() normally expects sea-level pressure and returns altitude above
// sea level. Handing it the ground reference instead makes the result altitude
// above the launch site, which is what the flight logic wants.
float baroReadAltitude() {
  return bmp.readAltitude(pGround);
}

// See Barometer_BMP280.h.
float baroReadPressure() {
  return bmp.readPressure() / 100.0;
}
