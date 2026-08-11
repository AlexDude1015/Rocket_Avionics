#ifndef FLIGHT_STATE_H
#define FLIGHT_STATE_H

#include <Arduino.h>

enum FlightState {
  STATE_BOOT,
  STATE_PAD,             // idle, then armed; may sit here for 3-5 hours
  STATE_BOOST,           // motor burning
  STATE_COAST,           // burnout to apogee
  STATE_APOGEE,          // drogue primary + backup fire here
  STATE_DROGUE_DESCENT,  // apogee down to main deploy altitude
  STATE_MAIN_DESCENT,    // main primary + backup fire on entry
  STATE_LANDED
};

const char *flightStateName(FlightState s);

#endif
