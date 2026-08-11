// ---------------------------------------------------------------------------
// Arduino B - backup flight computer
//
// Sensing, flight state machine and logging. Arduino B carries no radio, GPS
// or air brakes; those live on Arduino A.
//
// Deployment output is NOT implemented - see Pyro.h. The charges are tracked
// and logged, but nothing is actuated.
// ---------------------------------------------------------------------------

#include <SPI.h>
#include <math.h>
#include "Config.h"
#include "IMU_MPU6500.h"
#include "Barometer_BMP280.h"
#include "SDLogger.h"
#include "Orientation.h"
#include "Detectors.h"
#include "FlightState.h"
#include "Pyro.h"

static bool imuOK = false;
static bool baroOK = false;
static bool sdOK = false;

static FlightState state = STATE_BOOT;

// Latest sensor values, refreshed by the scheduler below.
static float ax_g = 0, ay_g = 0, az_g = 0;
static float gx_c = 0, gy_c = 0, gz_c = 0;
static float accelMagG = 1.0f;
static float axialG = 0;
static float altitudeM = 0;

// Arming
static bool     armed = false;
static bool     armWindowOpen = false;
static uint32_t armWindowStartMs = 0;
static float    armMagMin = 0, armMagMax = 0;

// Flight timestamps
static uint32_t tLiftoffMs = 0;
static uint32_t tApogeeMs = 0;
static uint32_t tMainDescentMs = 0;

// Scheduler
static uint32_t lastImuMs = 0, lastBaroMs = 0, lastLogMs = 0;
static uint32_t lastStatusMs = 0, lastFlushMs = 0, lastRebaseMs = 0;

// ---------------------------------------------------------------------------

static void transitionTo(FlightState s, uint32_t now) {
  state = s;
  Serial.print(">>> STATE -> ");
  Serial.println(flightStateName(s));
  sdLogEvent("STATE", flightStateName(s), altitudeM, now);

  if (s == STATE_LANDED) {
    sdClose();   // the whole point of detecting landing
    Serial.println("    log file closed");
  }
}

// Software preconditions only. The physical pull-pin switch is the real arming
// gate and is not wired to Arduino B, so this self-arms once the vehicle has
// been vertical and still long enough to trust the orientation capture.
static void updateArming(uint32_t now) {
  bool restart = false;

  if (!armWindowOpen) {
    restart = true;
  } else {
    if (accelMagG < armMagMin) armMagMin = accelMagG;
    if (accelMagG > armMagMax) armMagMax = accelMagG;

    if ((armMagMax - armMagMin) > ARM_QUIET_BAND_G) restart = true;
    if (fabs(accelMagG - 1.0f) > ARM_ACCEL_TOL_G)   restart = true;
  }

  if (restart) {
    armWindowOpen = true;
    armWindowStartMs = now;
    armMagMin = armMagMax = accelMagG;
    return;
  }

  if (now - armWindowStartMs < ARM_STATIONARY_MS) return;

  // The detectors cannot run without these two. SD is deliberately not
  // required: losing the log is bad, refusing to fly the sequence is worse.
  if (!imuOK || !baroOK) return;

  // TODO: pyro rail voltage sense, e-match continuity, CAN link health.

  if (!orientationCapture(ax_g, ay_g, az_g)) return;

  armed = true;

  float ux, uy, uz;
  orientationGetUp(ux, uy, uz);
  Serial.println("*** ARMED (software preconditions) ***");
  Serial.print("    up vector: ");
  Serial.print(ux, 3); Serial.print(", ");
  Serial.print(uy, 3); Serial.print(", ");
  Serial.println(uz, 3);

  sdLogEvent("ARMED", "software", altitudeM, now);
}

static void stepFlight(uint32_t now) {
  switch (state) {

    case STATE_PAD:
      if (!armed) {
        updateArming(now);
      } else if (liftoffUpdate(accelMagG, altitudeM, now)) {
        tLiftoffMs = now;
        baroFreezeGroundReference();   // stop chasing weather drift
        burnoutReset();
        apogeeReset();
        transitionTo(STATE_BOOST, now);
      }
      break;

    case STATE_BOOST:
      if (burnoutUpdate(axialG, now - tLiftoffMs, now)) {
        transitionTo(STATE_COAST, now);
      }
      break;

    case STATE_COAST:
      if (apogeeUpdate(altitudeM, now)) {
        tApogeeMs = now;
        Serial.print("    peak altitude: ");
        Serial.println(apogeePeakAltitude());
        transitionTo(STATE_APOGEE, now);
      }
      break;

    case STATE_APOGEE:
      if (now - tApogeeMs >= DROGUE_FIRE_DELAY_MS) {
        pyroFire(PYRO_DROGUE_PRIMARY, altitudeM, now);
      }
      if (now - tApogeeMs >= (DROGUE_FIRE_DELAY_MS + BACKUP_CHARGE_DELAY_MS)) {
        pyroFire(PYRO_DROGUE_BACKUP, altitudeM, now);
        transitionTo(STATE_DROGUE_DESCENT, now);
      }
      break;

    case STATE_DROGUE_DESCENT:
      if (altitudeM <= MAIN_DEPLOY_ALT_M) {
        tMainDescentMs = now;
        transitionTo(STATE_MAIN_DESCENT, now);
        pyroFire(PYRO_MAIN_PRIMARY, altitudeM, now);
        landingReset();
      }
      break;

    case STATE_MAIN_DESCENT:
      if (now - tMainDescentMs >= BACKUP_CHARGE_DELAY_MS) {
        pyroFire(PYRO_MAIN_BACKUP, altitudeM, now);
      }
      if (landingUpdate(altitudeM, accelMagG, now)) {
        transitionTo(STATE_LANDED, now);
      }
      break;

    case STATE_LANDED:
    case STATE_BOOT:
    default:
      break;
  }

  // Backstop: if landing is never detected the log file never closes, and a
  // disconnected battery then costs the entire flight record.
  if (state != STATE_LANDED && tLiftoffMs != 0 &&
      (now - tLiftoffMs) >= FLIGHT_TIMEOUT_MS) {
    sdLogEvent("TIMEOUT", "forced LANDED", altitudeM, now);
    transitionTo(STATE_LANDED, now);
  }
}

static void printStatus(uint32_t now) {
  Serial.print("[");
  Serial.print(flightStateName(state));
  if (state == STATE_PAD) Serial.print(armed ? "/ARMED" : "/IDLE");
  Serial.print("] alt=");
  Serial.print(altitudeM, 2);
  Serial.print("m |a|=");
  Serial.print(accelMagG, 3);
  Serial.print("g axial=");
  if (orientationValid()) Serial.print(axialG, 3);
  else Serial.print("--");
  Serial.print("g  t=");
  Serial.println(now);
}

// ---------------------------------------------------------------------------

void setup() {
  Serial.begin(SERIAL_BAUD);
  while (!Serial) delay(10);

  SPI.begin();
  delay(100);

  pyroInit();
  liftoffReset();
  burnoutReset();
  apogeeReset();
  landingReset();

  Serial.println("=== Avionics Bay (Arduino B) Self-Test ===");

  imuOK = imuBegin();
  Serial.print("IMU (MPU-6500): ");
  Serial.println(imuOK ? "PASS" : "FAIL");

  baroOK = baroBegin();
  Serial.print("Barometer (BMP280): ");
  Serial.println(baroOK ? "PASS" : "FAIL");

  sdOK = sdBegin();
  Serial.print("SD Card: ");
  Serial.println(sdOK ? "PASS" : "FAIL");

  if (imuOK) imuCalibrate();
  if (baroOK) baroCaptureGroundReference();

  Serial.println("=== Self-Test Complete ===");
  Serial.println("NOTE: pyro output is stubbed - nothing will be actuated.");

  uint32_t now = millis();
  lastImuMs = lastBaroMs = lastLogMs = now;
  lastStatusMs = lastFlushMs = lastRebaseMs = now;

  transitionTo(STATE_PAD, now);
}

void loop() {
  uint32_t now = millis();

  if (imuOK && (now - lastImuMs >= IMU_PERIOD_MS)) {
    lastImuMs = now;
    imuReadG(ax_g, ay_g, az_g, gx_c, gy_c, gz_c);
    accelMagG = imuAccelMagnitudeG(ax_g, ay_g, az_g);
    if (orientationValid()) {
      axialG = orientationAxial(ax_g, ay_g, az_g);
    }
  }

  if (baroOK && (now - lastBaroMs >= BARO_PERIOD_MS)) {
    lastBaroMs = now;
    altitudeM = baroReadAltitude();
  }

  stepFlight(now);

  if (baroOK && state == STATE_PAD &&
      (now - lastRebaseMs >= REBASELINE_PERIOD_MS)) {
    lastRebaseMs = now;
    baroRebaselineStep();
  }

  if (now - lastLogMs >= LOG_PERIOD_MS) {
    lastLogMs = now;
    sdLogSample(now, flightStateName(state),
                ax_g, ay_g, az_g, gx_c, gy_c, gz_c,
                altitudeM, accelMagG, axialG);
  }

  if (now - lastFlushMs >= SD_FLUSH_PERIOD_MS) {
    lastFlushMs = now;
    sdFlush();
  }

  if (now - lastStatusMs >= STATUS_PERIOD_MS) {
    lastStatusMs = now;
    printStatus(now);
  }
}
