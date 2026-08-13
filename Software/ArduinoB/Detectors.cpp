#include "Detectors.h"
#include "Config.h"
#include <math.h>

// ---------------------------------------------------------------------------
// Liftoff
// ---------------------------------------------------------------------------

static bool     liftoffLatched = false;  // event declared, stays true
static bool     liftoffTiming = false;   // currently timing a candidate
static uint32_t liftoffSinceMs = 0;      // when the candidate started

// See Detectors.h.
void liftoffReset() {
  liftoffLatched = false;
  liftoffTiming = false;
  liftoffSinceMs = 0;
}

// See Detectors.h.
// The timing pattern used here appears in three of the four detectors: note
// when a condition first became true, and only declare the event once it has
// held continuously long enough. The else-branch is the important half - any
// sample that breaks the condition abandons the candidate entirely.
bool liftoffUpdate(float accelMagG, float altitudeAglM, uint32_t nowMs) {
  if (liftoffLatched) return false;

  if (accelMagG >= LIFTOFF_ACCEL_G) {
    if (!liftoffTiming) {
      liftoffTiming = true;        // first sample above threshold, start timing
      liftoffSinceMs = nowMs;
    } else if (nowMs - liftoffSinceMs >= LIFTOFF_CONFIRM_MS) {
      liftoffLatched = true;       // held long enough to be a real boost
      return true;
    }
  } else {
    liftoffTiming = false;         // dipped below - a bump, not a launch
  }

  // Independent fallback path, checked even while the accel timer runs.
  if (altitudeAglM > LIFTOFF_BARO_BACKUP_M) {
    liftoffLatched = true;
    return true;
  }

  return false;
}

// See Detectors.h.
bool liftoffDetected() {
  return liftoffLatched;
}

// ---------------------------------------------------------------------------
// Burnout
// ---------------------------------------------------------------------------

static bool     burnoutLatched = false;
static bool     burnoutTiming = false;
static uint32_t burnoutSinceMs = 0;

// See Detectors.h.
void burnoutReset() {
  burnoutLatched = false;
  burnoutTiming = false;
  burnoutSinceMs = 0;
}

// See Detectors.h.
// The threshold sits slightly below zero rather than at it, so noise around the
// crossing cannot trip it early. The confirmation window rides through thrust
// tail-off and chuffing, where a motor may briefly stutter before it is truly
// finished burning.
bool burnoutUpdate(float axialG, uint32_t tSinceLiftoffMs, uint32_t nowMs) {
  if (burnoutLatched) return false;

  if (axialG <= BURNOUT_AXIAL_G) {
    if (!burnoutTiming) {
      burnoutTiming = true;
      burnoutSinceMs = nowMs;
    } else if (nowMs - burnoutSinceMs >= BURNOUT_CONFIRM_MS) {
      burnoutLatched = true;
      return true;
    }
  } else {
    burnoutTiming = false;         // still pushing forward
  }

  // Fallback: never sit in BOOST indefinitely, whatever the sensors say.
  if (tSinceLiftoffMs >= BURNOUT_TIMEOUT_MS) {
    burnoutLatched = true;
    return true;
  }

  return false;
}

// See Detectors.h.
bool burnoutDetected() {
  return burnoutLatched;
}

// ---------------------------------------------------------------------------
// Apogee
// ---------------------------------------------------------------------------

static bool  apogeeLatched = false;
static bool  apogeeSeeded = false;   // has the peak been given a starting value
static float apogeePeakM = 0;        // highest altitude seen since reset

// See Detectors.h.
void apogeeReset() {
  apogeeLatched = false;
  apogeeSeeded = false;
  apogeePeakM = 0;
}

// See Detectors.h.
// Three cases: still climbing (raise the peak), fallen far enough below the
// peak (apogee), or dropped a little but not convincingly (wait).
bool apogeeUpdate(float altitudeM, uint32_t nowMs) {
  (void)nowMs;                       // unused; interface kept uniform
  if (apogeeLatched) return false;

  // Seed from the first real reading rather than assuming zero, so the peak
  // starts somewhere sensible whatever altitude coast begins at.
  if (!apogeeSeeded) {
    apogeeSeeded = true;
    apogeePeakM = altitudeM;
    return false;
  }

  if (altitudeM > apogeePeakM) {
    apogeePeakM = altitudeM;         // new high point
    return false;
  }

  // Measured against the running peak, not the previous sample: noise wobbles
  // without accumulating, so only a genuine descent builds up this much drop.
  if ((apogeePeakM - altitudeM) >= APOGEE_HYSTERESIS_M) {
    apogeeLatched = true;
    return true;
  }

  return false;
}

// See Detectors.h.
bool apogeeDetected() {
  return apogeeLatched;
}

// See Detectors.h.
float apogeePeakAltitude() {
  return apogeePeakM;
}

// ---------------------------------------------------------------------------
// Landing
// ---------------------------------------------------------------------------

static bool     landingLatched = false;
static bool     landingWindowOpen = false;
static uint32_t landingWindowStartMs = 0;

// Extremes seen since the window opened. Comparing highest against lowest
// measures how much the value has moved, without storing a history buffer.
static float    landAltMin = 0, landAltMax = 0;
static float    landMagMin = 0, landMagMax = 0;

// See Detectors.h.
void landingReset() {
  landingLatched = false;
  landingWindowOpen = false;
  landingWindowStartMs = 0;
}

// See Detectors.h.
// Same sustained-condition idea as liftoff and burnout, but tracking spread
// rather than a threshold: any sample that widens either range beyond its
// allowance restarts the window from scratch.
bool landingUpdate(float altitudeM, float accelMagG, uint32_t nowMs) {
  if (landingLatched) return false;

  bool restart = false;

  if (!landingWindowOpen) {
    restart = true;                  // first call, open a window
  } else {
    if (altitudeM < landAltMin) landAltMin = altitudeM;
    if (altitudeM > landAltMax) landAltMax = altitudeM;
    if (accelMagG < landMagMin) landMagMin = accelMagG;
    if (accelMagG > landMagMax) landMagMax = accelMagG;

    if ((landAltMax - landAltMin) > LANDING_ALT_BAND_M)   restart = true;  // moving
    if ((landMagMax - landMagMin) > LANDING_QUIET_BAND_G) restart = true;  // shaking
    if (fabs(accelMagG - 1.0f) > LANDING_ACCEL_TOL_G)     restart = true;  // not at rest
  }

  if (restart) {
    landingWindowOpen = true;
    landingWindowStartMs = nowMs;
    landAltMin = landAltMax = altitudeM;
    landMagMin = landMagMax = accelMagG;
    return false;
  }

  if (nowMs - landingWindowStartMs >= LANDING_WINDOW_MS) {
    landingLatched = true;
    return true;
  }

  return false;
}

// See Detectors.h.
bool landingDetected() {
  return landingLatched;
}
