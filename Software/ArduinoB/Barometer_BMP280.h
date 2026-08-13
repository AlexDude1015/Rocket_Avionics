#ifndef BAROMETER_BMP280_H
#define BAROMETER_BMP280_H

#include <Arduino.h>

// ---------------------------------------------------------------------------
// BMP280 barometric pressure sensor, driven over SPI via the Adafruit driver.
//
// Air pressure falls predictably with height, so the sensor doubles as an
// altimeter. It cannot know how high the launch site itself is, so a "ground
// reference" pressure is captured while the vehicle sits on the pad and every
// altitude afterwards is reported relative to it - that is, altitude above
// ground level (AGL), which is what the deployment logic needs.
//
// This is the primary sensor for apogee detection and for the 950 ft main
// deployment trigger.
// ---------------------------------------------------------------------------

// Initialises the sensor over SPI.
// Must be called before anything else here.
// Returns true on success, false if the sensor does not respond - usually a
// wiring, power or chip-select fault.
bool baroBegin();

// Captures the launch site's pressure as the zero-altitude reference by
// averaging 20 readings, which smooths out sensor noise.
// Blocks for about 2 seconds, so this runs once at startup, and the vehicle
// should be stationary at ground level while it does.
void baroCaptureGroundReference();

// Nudges the ground reference toward the current pressure by a very small
// amount per call.
//
// Why this exists: weather moves air pressure over hours, and roughly 1 hPa of
// drift looks like 28 ft of altitude change. Across a 3-5 hour hold on the pad
// that would leave the reference badly stale, throwing off the 950 ft main
// deployment altitude. Correcting slowly keeps it honest.
//
// Only call this while stationary and before liftoff. It is a no-op once the
// reference has been frozen.
void baroRebaselineStep();

// Locks the ground reference so it stops tracking. Called the instant liftoff
// is detected, because from then on every pressure change is real altitude and
// must not be mistaken for weather.
void baroFreezeGroundReference();

// Returns true once the reference has been frozen by baroFreezeGroundReference().
bool baroGroundReferenceFrozen();

// Returns the current ground reference pressure in hPa.
float baroGroundPressure();

// Returns height above the captured ground reference, in metres (AGL).
// Reads about 0 on the pad, and is the value apogee and main-deploy detection
// both work from.
float baroReadAltitude();

// Returns the raw atmospheric pressure in hPa, with no reference applied.
float baroReadPressure();

#endif
