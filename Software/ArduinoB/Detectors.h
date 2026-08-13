#ifndef DETECTORS_H
#define DETECTORS_H

#include <Arduino.h>

// ---------------------------------------------------------------------------
// The four flight events that drive the state machine, each detected from the
// sensor streams.
//
// Shared behaviour worth knowing before reading any of them:
//
// LATCHING - once an event is declared it stays declared. update() returns
// true exactly once, on the transition itself, and false on every later call.
// This stops the flight sequence ever running backwards, which sensor noise
// could otherwise cause at a threshold boundary.
//
// SUSTAINED CONDITIONS - the timing-based detectors require a condition to
// hold continuously for a set period, not just appear in one sample. A single
// noisy reading proves nothing; a condition that survives 150 ms is real. Any
// sample that breaks the condition restarts the clock.
// ---------------------------------------------------------------------------

// --- Liftoff (PAD -> BOOST) -------------------------------------------------

// Clears the detector back to its pre-flight state.
void liftoffReset();

// Tests whether the rocket has left the pad.
//   accelMagG    - total acceleration magnitude in g
//   altitudeAglM - current height above the pad, in metres
//   nowMs        - current millis() timestamp
// Returns true once, on the transition.
//
// Uses acceleration MAGNITUDE rather than a single axis deliberately: missing
// liftoff is the most catastrophic failure in the whole sequence (no
// deployment at all), so its detector carries no dependency on the orientation
// capture having worked. Magnitude reads ~1g at rest in any attitude.
//
// The altitude argument is a fallback, ORed in rather than ANDed: if the
// accelerometer path fails entirely, climbing past the backup height still
// starts the flight sequence, late but functional. The bias is intentionally
// toward detecting.
bool liftoffUpdate(float accelMagG, float altitudeAglM, uint32_t nowMs);

// Returns true if liftoff has been declared.
bool liftoffDetected();

// --- Burnout (BOOST -> COAST) -----------------------------------------------

// Clears the detector. Called when entering BOOST.
void burnoutReset();

// Tests whether the motor has stopped thrusting.
//   axialG           - signed acceleration along the flight axis, in g
//   tSinceLiftoffMs  - milliseconds elapsed since liftoff, for the fallback
//   nowMs            - current millis() timestamp
// Returns true once, on the transition.
//
// Detected as a SIGN CHANGE, not a drop to zero. Because the accelerometer
// ignores gravity, coasting flight reads pure drag, which pushes backward -
// so the axial reading swings from strongly positive under thrust to negative
// in coast. That crossing sits at zero regardless of how fast the vehicle is
// travelling, whereas any magnitude threshold would depend on a burnout speed
// that is not known in advance.
//
// The elapsed-time argument is a fallback so that a bad orientation capture
// cannot strand the flight in BOOST forever.
bool burnoutUpdate(float axialG, uint32_t tSinceLiftoffMs, uint32_t nowMs);

// Returns true if burnout has been declared.
bool burnoutDetected();

// --- Apogee (COAST -> APOGEE) -----------------------------------------------

// Clears the detector and forgets the running peak.
//
// Call this when coast begins, NOT at boot. Air pressure drifts with weather
// over the hours spent waiting on the pad, and a detector left running would
// eventually read that drift as a descent and declare a false apogee while
// still on the ground.
void apogeeReset();

// Tests whether the rocket has passed its highest point.
//   altitudeM - current altitude in metres
//   nowMs     - current millis() timestamp (unused; kept for a uniform interface)
// Returns true once, on the transition.
//
// Works by remembering the highest altitude seen so far and waiting for the
// current reading to fall a set margin BELOW it - a technique called
// hysteresis. Firing on the first lower-than-last reading would trigger
// constantly on sensor noise near the top, where the real signal is smallest.
// Requiring a cumulative drop works because noise wobbles both ways without
// accumulating, whereas a genuine descent keeps going.
bool apogeeUpdate(float altitudeM, uint32_t nowMs);

// Returns true if apogee has been declared.
bool apogeeDetected();

// Returns the highest altitude recorded, in metres - the flight's apogee.
float apogeePeakAltitude();

// --- Landing (MAIN DESCENT -> LANDED) ---------------------------------------

// Clears the detector. Called when the main parachute deploys.
void landingReset();

// Tests whether the rocket has come to rest on the ground.
//   altitudeM  - current altitude in metres
//   accelMagG  - total acceleration magnitude in g
//   nowMs      - current millis() timestamp
// Returns true once, on the transition.
//
// Requires BOTH altitude and acceleration to stay quiet for a sustained window.
// They are not redundant:
//
//   Altitude does the real work. Acceleration alone cannot tell landed from
//   still-descending, because a parachute at steady speed also reads ~1g -
//   drag and gravity cancel exactly, just as they do sitting on the ground.
//
//   Acceleration guards against a failed barometer. A frozen or blocked sensor
//   reports perfectly stable altitude, which would otherwise look like a
//   landing in mid-air. A descending rocket still swings and shakes, so
//   checking that the vehicle is genuinely motionless catches that case.
bool landingUpdate(float altitudeM, float accelMagG, uint32_t nowMs);

// Returns true if landing has been declared.
bool landingDetected();

#endif
