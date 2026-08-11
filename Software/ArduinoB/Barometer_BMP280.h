#ifndef BAROMETER_BMP280_H
#define BAROMETER_BMP280_H

#include <Arduino.h>

bool baroBegin();

// Blocking, ~2 s. Runs once at boot while the vehicle is stationary.
void baroCaptureGroundReference();

// Barometric pressure drifts with weather over a 3-5 hour pad hold - roughly
// 28 ft of apparent altitude per hPa - which would leave the 950 ft main deploy
// reference stale by launch. Call this slowly while confirmed stationary and
// pre-liftoff, then freeze it the instant boost is detected.
void baroRebaselineStep();
void baroFreezeGroundReference();
bool baroGroundReferenceFrozen();

float baroGroundPressure();

// Altitude above the captured ground reference (AGL).
float baroReadAltitude();
float baroReadPressure();

#endif
