#ifndef FLIGHT_STATE_H
#define FLIGHT_STATE_H

#include <Arduino.h>

// ---------------------------------------------------------------------------
// The phases of flight, in the order they occur.
//
// Tracking the phase explicitly is what keeps each detector confined to the
// part of the flight where it makes sense - apogee detection must not run
// while sitting on the pad, deployment must not be possible during ascent, and
// so on. It also makes the flight log far easier to read afterwards, since
// every sample records which phase it belonged to.
// ---------------------------------------------------------------------------

enum FlightState {
  STATE_BOOT,            // powering up, sensor self-test and calibration
  STATE_PAD,             // waiting to launch, possibly for hours; arms here
  STATE_BOOST,           // motor burning
  STATE_COAST,           // burnout to apogee, still climbing but slowing
  STATE_APOGEE,          // highest point reached; drogue charges fire here
  STATE_DROGUE_DESCENT,  // falling fast under the small chute
  STATE_MAIN_DESCENT,    // main chute out, slow descent to the ground
  STATE_LANDED           // on the ground; log file closed
};

// Converts a state to a short human-readable name for serial output and the
// flight log.
//   s - the state to name
// Returns a static string, or "UNKNOWN" for an unrecognised value.
const char *flightStateName(FlightState s);

#endif
