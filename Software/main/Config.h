#ifndef CONFIG_H
#define CONFIG_H

// ---------------------------------------------------------------------------
// Arduino B (backup flight computer)
// ---------------------------------------------------------------------------

#define IMU_CS_PIN 3
#define BARO_CS_PIN 2
#define SD_CS_PIN 4

#define SERIAL_BAUD 9600

#define GROUND_LOG_FILENAME "flight.csv"

// ---------------------------------------------------------------------------
// Sample rates (non-blocking scheduler periods, milliseconds)
// ---------------------------------------------------------------------------

#define IMU_PERIOD_MS      10     // 100 Hz - liftoff/burnout need fast samples
#define BARO_PERIOD_MS     40     // 25 Hz
#define LOG_PERIOD_MS      50     // 20 Hz
#define STATUS_PERIOD_MS   1000   // human-readable serial output
#define SD_FLUSH_PERIOD_MS 1000   // flush cadence while the file stays open
#define REBASELINE_PERIOD_MS 1000 // ground-pressure drift correction on the pad

// ---------------------------------------------------------------------------
// IMU scaling - ACCEL_CONFIG is set to 0x18 (+/-16g) => 2048 LSB/g
// ---------------------------------------------------------------------------

#define ACCEL_LSB_PER_G 2048.0f

// ---------------------------------------------------------------------------
// Arming preconditions
// ---------------------------------------------------------------------------

#define ARM_STATIONARY_MS   3000   // must be still this long before arming
#define ARM_ACCEL_TOL_G     0.15f  // |a| must sit within 1g +/- this
#define ARM_QUIET_BAND_G    0.10f  // max peak-to-peak jitter while arming

// ---------------------------------------------------------------------------
// Liftoff detection (PAD -> BOOST)
//   Acceleration MAGNITUDE, deliberately orientation-independent so that the
//   most catastrophic-to-miss event does not depend on the orientation capture.
// ---------------------------------------------------------------------------

#define LIFTOFF_ACCEL_G       3.0f   // between 1g on the pad and ~11g boost
#define LIFTOFF_CONFIRM_MS    150    // rejects bumps; real boost holds seconds
#define LIFTOFF_BARO_BACKUP_M 61.0f  // ~200 ft - fallback if the accel path fails

// ---------------------------------------------------------------------------
// Burnout detection (BOOST -> COAST)
//   Signed AXIAL acceleration. The accelerometer does not sense gravity, so in
//   coast it reads drag alone, pointing aft => the sign flips at burnout.
// ---------------------------------------------------------------------------

#define BURNOUT_AXIAL_G    -0.5f  // margin past zero so noise cannot trip it
#define BURNOUT_CONFIRM_MS  200   // rides through thrust tail-off and chuffing
#define BURNOUT_TIMEOUT_MS  8000  // TODO: set to motor burn time + margin

// ---------------------------------------------------------------------------
// Apogee detection (COAST -> APOGEE)
//   Barometric running peak with hysteresis. Noise wobbles but does not
//   accumulate directionally, so a net drop below the peak is a real descent.
// ---------------------------------------------------------------------------

#define APOGEE_HYSTERESIS_M 6.0f  // ~20 ft below the running peak

// ---------------------------------------------------------------------------
// Deployment
//   All four charges fire unconditionally on phase-gated timers - no
//   deployment-success detection - so no live black powder survives landing.
// ---------------------------------------------------------------------------

#define DROGUE_FIRE_DELAY_MS   2000    // after apogee is declared
#define BACKUP_CHARGE_DELAY_MS 2000    // backup after its matching primary
#define MAIN_DEPLOY_ALT_M      289.6f  // 950 ft AGL

// ---------------------------------------------------------------------------
// Landing detection (MAIN DESCENT -> LANDED)
//   Altitude stability does the discriminating: under a chute at terminal
//   velocity the accelerometer also reads ~1g. The accel check is a
//   cross-check against a frozen/failed barometer.
// ---------------------------------------------------------------------------

#define LANDING_ALT_BAND_M   2.0f   // bench noise measured ~0.4 m peak-to-peak
#define LANDING_WINDOW_MS    10000  // main descent covers ~50 m in this window
#define LANDING_ACCEL_TOL_G  0.15f
#define LANDING_QUIET_BAND_G 0.10f  // swinging under a chute exceeds this

// Backstop: if LANDED never triggers the log file never closes, and a yanked
// battery then costs the whole flight record. Flight is ~5 min, so 10 is safe.
#define FLIGHT_TIMEOUT_MS 600000

#endif
