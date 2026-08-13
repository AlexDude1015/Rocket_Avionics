// ---------------------------------------------------------------------------
// Arduino B - backup flight computer
//
// Reads the sensors, works out which phase of flight the rocket is in, and
// records everything to the SD card. Arduino B carries no radio, GPS or air
// brakes; those live on Arduino A. The two boards run independently so that
// either can carry the flight on its own.
//
// Deployment output is NOT implemented - see Pyro.h. The charges are tracked
// and logged, but nothing is actuated.
//
// HOW THE PROGRAM IS ORGANISED
//
// setup() runs the sensor self-test and calibration once, then hands over to
// loop(), which runs continuously and does three things each pass:
//
//   1. Sample the sensors, but only when each one is due. The IMU is read 100
//      times a second, the barometer 25. This is done by comparing timestamps
//      rather than calling delay(), because delay() stops the processor
//      completely - and while it slept, a launch could come and go unnoticed.
//
//   2. Step the flight state machine, which asks the detector for whichever
//      event is next expected and moves the state on when it fires.
//
//   3. Write to the log and print a status line, again each on its own
//      schedule.
//
// Nothing in the loop blocks, so the fastest task always keeps its rate no
// matter what else is happening.
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

// Which hardware passed its self-test. Checked before using each sensor, so a
// single failed device does not take the whole flight computer down with it.
static bool imuOK = false;
static bool baroOK = false;
static bool sdOK = false;

static FlightState state = STATE_BOOT;

// Most recent sensor values, refreshed by the scheduler in loop() and read by
// everything else. Kept here so the detectors, logger and status print all see
// the same numbers within a pass.
static float ax_g = 0, ay_g = 0, az_g = 0;   // acceleration, g
static float gx_c = 0, gy_c = 0, gz_c = 0;   // gyroscope, bias-corrected
static float accelMagG = 1.0f;               // total acceleration magnitude, g
static float axialG = 0;                     // acceleration along flight axis, g
static float altitudeM = 0;                  // height above the pad, metres

// Arming. The quiet-window variables track how long the vehicle has been
// genuinely still, using the same restart-on-disturbance idea as the detectors.
static bool     armed = false;
static bool     armWindowOpen = false;
static uint32_t armWindowStartMs = 0;
static float    armMagMin = 0, armMagMax = 0;

// Reference times for phase-gated actions. Deployment is timed from these
// rather than from power-on, so nothing can fire before its phase is reached.
static uint32_t tLiftoffMs = 0;
static uint32_t tApogeeMs = 0;
static uint32_t tMainDescentMs = 0;

// When each periodic task last ran. Comparing these against millis() is what
// gives each task its own rate without ever blocking.
static uint32_t lastImuMs = 0, lastBaroMs = 0, lastLogMs = 0;
static uint32_t lastStatusMs = 0, lastFlushMs = 0, lastRebaseMs = 0;

// ---------------------------------------------------------------------------

// Moves to a new flight phase, recording it to serial and the log.
//   s   - the phase being entered
//   now - current millis() timestamp
// Every transition is logged, because reconstructing afterwards why a charge
// fired when it did is far easier from a phase record than from raw sensor
// traces. Reaching LANDED also closes the log file, which is the whole reason
// landing is detected at all.
static void transitionTo(FlightState s, uint32_t now) {
  state = s;
  Serial.print(">>> STATE -> ");
  Serial.println(flightStateName(s));
  sdLogEvent("STATE", flightStateName(s), altitudeM, now);

  if (s == STATE_LANDED) {
    sdClose();
    Serial.println("    log file closed");
  }
}

// Decides whether the vehicle is ready to fly, and arms it if so.
//   now - current millis() timestamp
//
// Arming is the point where the flight computer starts watching for launch,
// and where it learns its own orientation. Both need the rocket upright and
// still, which is exactly the situation on the launch rail.
//
// This checks software preconditions only. The physical pull-pin switch is the
// real arming gate and is not wired to Arduino B, so this self-arms once the
// vehicle has been vertical and still for long enough to be trusted.
static void updateArming(uint32_t now) {
  bool restart = false;

  if (!armWindowOpen) {
    restart = true;
  } else {
    // Track how much the reading has wandered since the window opened.
    if (accelMagG < armMagMin) armMagMin = accelMagG;
    if (accelMagG > armMagMax) armMagMax = accelMagG;

    if ((armMagMax - armMagMin) > ARM_QUIET_BAND_G) restart = true;  // jostled
    if (fabs(accelMagG - 1.0f) > ARM_ACCEL_TOL_G)   restart = true;  // not at rest
  }

  // Any disturbance restarts the clock, so the vehicle must be left alone for
  // the full period before it will arm.
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

  // Learn which way is up. Refusing to arm on failure is deliberate - a wrong
  // up-vector would silently corrupt burnout detection later in the flight.
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

// Runs one step of the flight state machine.
//   now - current millis() timestamp
//
// Each phase watches for exactly one thing, so a detector can never fire
// outside the part of the flight where it makes sense. Called every pass of
// loop(); most calls do nothing but check a condition.
static void stepFlight(uint32_t now) {
  switch (state) {

    // Waiting to launch. Arms first, then watches for liftoff - detection is
    // gated on being armed, which rules out false triggers from handling.
    case STATE_PAD:
      if (!armed) {
        updateArming(now);
      } else if (liftoffUpdate(accelMagG, altitudeM, now)) {
        tLiftoffMs = now;                // the flight clock starts here
        baroFreezeGroundReference();     // pressure changes are real altitude now
        burnoutReset();
        apogeeReset();                   // only starts evaluating from here
        transitionTo(STATE_BOOST, now);
      }
      break;

    // Motor burning. Waiting for thrust to end.
    case STATE_BOOST:
      if (burnoutUpdate(axialG, now - tLiftoffMs, now)) {
        transitionTo(STATE_COAST, now);
      }
      break;

    // Climbing without power, slowing under drag and gravity. On Arduino A
    // this is where the air brakes would work.
    case STATE_COAST:
      if (apogeeUpdate(altitudeM, now)) {
        tApogeeMs = now;
        Serial.print("    peak altitude: ");
        Serial.println(apogeePeakAltitude());
        transitionTo(STATE_APOGEE, now);
      }
      break;

    // Highest point. Both drogue charges fire here on a short delay: the
    // primary first, then the backup regardless of whether the primary worked.
    // Firing the backup unconditionally guarantees no live charge remains.
    case STATE_APOGEE:
      if (now - tApogeeMs >= DROGUE_FIRE_DELAY_MS) {
        pyroFire(PYRO_DROGUE_PRIMARY, altitudeM, now);
      }
      if (now - tApogeeMs >= (DROGUE_FIRE_DELAY_MS + BACKUP_CHARGE_DELAY_MS)) {
        pyroFire(PYRO_DROGUE_BACKUP, altitudeM, now);
        transitionTo(STATE_DROGUE_DESCENT, now);
      }
      break;

    // Falling fast under the drogue. The main is held back until low altitude
    // so the rocket does not drift far on the wind under a big canopy.
    case STATE_DROGUE_DESCENT:
      if (altitudeM <= MAIN_DEPLOY_ALT_M) {
        tMainDescentMs = now;
        transitionTo(STATE_MAIN_DESCENT, now);
        pyroFire(PYRO_MAIN_PRIMARY, altitudeM, now);
        landingReset();
      }
      break;

    // Main chute out. Fire its backup, then watch for touchdown.
    case STATE_MAIN_DESCENT:
      if (now - tMainDescentMs >= BACKUP_CHARGE_DELAY_MS) {
        pyroFire(PYRO_MAIN_BACKUP, altitudeM, now);
      }
      if (landingUpdate(altitudeM, accelMagG, now)) {
        transitionTo(STATE_LANDED, now);
      }
      break;

    // Nothing left to do; the log is closed.
    case STATE_LANDED:
    case STATE_BOOT:
    default:
      break;
  }

  // Backstop: if landing is never detected the log file never closes, and a
  // disconnected battery then costs the entire flight record. The flight lasts
  // about five minutes, so this cannot fire early.
  if (state != STATE_LANDED && tLiftoffMs != 0 &&
      (now - tLiftoffMs) >= FLIGHT_TIMEOUT_MS) {
    sdLogEvent("TIMEOUT", "forced LANDED", altitudeM, now);
    transitionTo(STATE_LANDED, now);
  }
}

// Prints a one-line summary for whoever is watching the serial monitor.
//   now - current millis() timestamp
// The axial figure shows "--" until orientation has been captured, since it
// has no meaning before then.
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

// Runs once at power-on: brings up the sensors, checks each one, and performs
// the two calibrations that need the vehicle to be still.
//
// Both calibrations block for a second or more, which is fine here but would
// not be during flight - hence doing them once, up front.
void setup() {
  Serial.begin(SERIAL_BAUD);
  while (!Serial) delay(10);

  SPI.begin();
  delay(100);              // let the sensor power rails settle

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

  if (imuOK) imuCalibrate();                 // gyro zero offset, ~1 s
  if (baroOK) baroCaptureGroundReference();  // pad pressure, ~2 s

  Serial.println("=== Self-Test Complete ===");
  Serial.println("NOTE: pyro output is stubbed - nothing will be actuated.");

  // Start every schedule from now, so nothing thinks it is already overdue.
  uint32_t now = millis();
  lastImuMs = lastBaroMs = lastLogMs = now;
  lastStatusMs = lastFlushMs = lastRebaseMs = now;

  transitionTo(STATE_PAD, now);
}

// Runs continuously. Each block checks whether its task is due and skips it
// otherwise, so a slow task can never delay a fast one.
//
// The pattern "now - lastRun >= period" is used throughout. Subtracting this
// way is deliberate: it stays correct even when millis() wraps back to zero
// after about 49 days of running.
void loop() {
  uint32_t now = millis();

  // Fastest sensor. Liftoff and burnout both need a condition to hold for a
  // couple of hundred milliseconds, which takes many samples to establish.
  if (imuOK && (now - lastImuMs >= IMU_PERIOD_MS)) {
    lastImuMs = now;
    imuReadG(ax_g, ay_g, az_g, gx_c, gy_c, gz_c);
    accelMagG = imuAccelMagnitudeG(ax_g, ay_g, az_g);
    if (orientationValid()) {
      axialG = orientationAxial(ax_g, ay_g, az_g);
    }
  }

  // The barometer is slower and noisier than the IMU, and apogee detection
  // does not need it any faster than this.
  if (baroOK && (now - lastBaroMs >= BARO_PERIOD_MS)) {
    lastBaroMs = now;
    altitudeM = baroReadAltitude();
  }

  stepFlight(now);

  // Correct the ground reference for weather drift, but only while still on
  // the pad. It is frozen at liftoff, so this stops on its own.
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

  // Commit buffered data periodically, bounding what a sudden power loss
  // could take with it.
  if (now - lastFlushMs >= SD_FLUSH_PERIOD_MS) {
    lastFlushMs = now;
    sdFlush();
  }

  if (now - lastStatusMs >= STATUS_PERIOD_MS) {
    lastStatusMs = now;
    printStatus(now);
  }
}
