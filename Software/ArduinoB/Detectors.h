#ifndef DETECTORS_H
#define DETECTORS_H

#include <Arduino.h>

// Flight event detection from the sensor streams. Every detector latches: once
// an event is declared it stays declared, so the state machine can never fall
// backwards mid-flight. Each update() returns true exactly once, on the
// transition, and false on every later call.

// --- Liftoff (PAD -> BOOST) -------------------------------------------------
// Acceleration magnitude, so this does not depend on the orientation capture.
// The barometric argument is an OR fallback: missing liftoff is unrecoverable,
// so this deliberately biases toward detecting.
void liftoffReset();
bool liftoffUpdate(float accelMagG, float altitudeAglM, uint32_t nowMs);
bool liftoffDetected();

// --- Burnout (BOOST -> COAST) -----------------------------------------------
// Signed axial acceleration crossing negative. tSinceLiftoffMs drives the
// timeout fallback so a bad orientation capture cannot strand the flight in
// BOOST forever.
void burnoutReset();
bool burnoutUpdate(float axialG, uint32_t tSinceLiftoffMs, uint32_t nowMs);
bool burnoutDetected();

// --- Apogee (COAST -> APOGEE) -----------------------------------------------
// Barometric running peak with hysteresis. Call reset() when coast begins, not
// at boot: hours of weather drift on the pad would false-trigger it.
void apogeeReset();
bool apogeeUpdate(float altitudeM, uint32_t nowMs);
bool apogeeDetected();
float apogeePeakAltitude();

// --- Landing (MAIN DESCENT -> LANDED) ---------------------------------------
// Altitude stability discriminates; the accel check guards against a frozen
// barometer, which would otherwise look perfectly "stable" mid-descent.
void landingReset();
bool landingUpdate(float altitudeM, float accelMagG, uint32_t nowMs);
bool landingDetected();

#endif
