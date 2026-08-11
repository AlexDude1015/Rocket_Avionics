#include "FlightState.h"

const char *flightStateName(FlightState s) {
  switch (s) {
    case STATE_BOOT:            return "BOOT";
    case STATE_PAD:             return "PAD";
    case STATE_BOOST:           return "BOOST";
    case STATE_COAST:           return "COAST";
    case STATE_APOGEE:          return "APOGEE";
    case STATE_DROGUE_DESCENT:  return "DROGUE_DESCENT";
    case STATE_MAIN_DESCENT:    return "MAIN_DESCENT";
    case STATE_LANDED:          return "LANDED";
  }
  return "UNKNOWN";
}
