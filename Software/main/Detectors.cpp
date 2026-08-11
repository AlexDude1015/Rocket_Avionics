#include "Detectors.h"
#include "Config.h"
#include <math.h>

// ---------------------------------------------------------------------------
// Liftoff
// ---------------------------------------------------------------------------

static bool     liftoffLatched = false;
static bool     liftoffTiming = false;
static uint32_t liftoffSinceMs = 0;

void liftoffReset() {
  liftoffLatched = false;
  liftoffTiming = false;
  liftoffSinceMs = 0;
}

bool liftoffUpdate(float accelMagG, float altitudeAglM, uint32_t nowMs) {
  if (liftoffLatched) return false;

  if (accelMagG >= LIFTOFF_ACCEL_G) {
    if (!liftoffTiming) {
      liftoffTiming = true;
      liftoffSinceMs = nowMs;
    } else if (nowMs - liftoffSinceMs >= LIFTOFF_CONFIRM_MS) {
      liftoffLatched = true;
      return true;
    }
  } else {
    liftoffTiming = false;  // must be continuous
  }

  if (altitudeAglM > LIFTOFF_BARO_BACKUP_M) {
    liftoffLatched = true;
    return true;
  }

  return false;
}

bool liftoffDetected() {
  return liftoffLatched;
}

// ---------------------------------------------------------------------------
// Burnout
// ---------------------------------------------------------------------------

static bool     burnoutLatched = false;
static bool     burnoutTiming = false;
static uint32_t burnoutSinceMs = 0;

void burnoutReset() {
  burnoutLatched = false;
  burnoutTiming = false;
  burnoutSinceMs = 0;
}

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
    burnoutTiming = false;
  }

  if (tSinceLiftoffMs >= BURNOUT_TIMEOUT_MS) {
    burnoutLatched = true;
    return true;
  }

  return false;
}

bool burnoutDetected() {
  return burnoutLatched;
}

// ---------------------------------------------------------------------------
// Apogee
// ---------------------------------------------------------------------------

static bool  apogeeLatched = false;
static bool  apogeeSeeded = false;
static float apogeePeakM = 0;

void apogeeReset() {
  apogeeLatched = false;
  apogeeSeeded = false;
  apogeePeakM = 0;
}

bool apogeeUpdate(float altitudeM, uint32_t nowMs) {
  (void)nowMs;
  if (apogeeLatched) return false;

  if (!apogeeSeeded) {
    apogeeSeeded = true;
    apogeePeakM = altitudeM;
    return false;
  }

  if (altitudeM > apogeePeakM) {
    apogeePeakM = altitudeM;   // still climbing, or noise near the top
    return false;
  }

  if ((apogeePeakM - altitudeM) >= APOGEE_HYSTERESIS_M) {
    apogeeLatched = true;
    return true;
  }

  return false;  // dropped a little, not enough to be sure yet
}

bool apogeeDetected() {
  return apogeeLatched;
}

float apogeePeakAltitude() {
  return apogeePeakM;
}

// ---------------------------------------------------------------------------
// Landing
// ---------------------------------------------------------------------------

static bool     landingLatched = false;
static bool     landingWindowOpen = false;
static uint32_t landingWindowStartMs = 0;
static float    landAltMin = 0, landAltMax = 0;
static float    landMagMin = 0, landMagMax = 0;

void landingReset() {
  landingLatched = false;
  landingWindowOpen = false;
  landingWindowStartMs = 0;
}

bool landingUpdate(float altitudeM, float accelMagG, uint32_t nowMs) {
  if (landingLatched) return false;

  bool restart = false;

  if (!landingWindowOpen) {
    restart = true;
  } else {
    if (altitudeM < landAltMin) landAltMin = altitudeM;
    if (altitudeM > landAltMax) landAltMax = altitudeM;
    if (accelMagG < landMagMin) landMagMin = accelMagG;
    if (accelMagG > landMagMax) landMagMax = accelMagG;

    if ((landAltMax - landAltMin) > LANDING_ALT_BAND_M)   restart = true;
    if ((landMagMax - landMagMin) > LANDING_QUIET_BAND_G) restart = true;
    if (fabs(accelMagG - 1.0f) > LANDING_ACCEL_TOL_G)     restart = true;
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

bool landingDetected() {
  return landingLatched;
}
