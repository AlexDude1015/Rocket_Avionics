#ifndef PYRO_H
#define PYRO_H

#include <Arduino.h>

// ---------------------------------------------------------------------------
// SAFETY: THIS MODULE DOES NOT ACTUATE ANYTHING.
//
// Whether Arduino B drives the relay board at all is still an open hardware
// decision, so no relay pins are defined here and nothing can be energized.
// pyroFire() records the event to serial and the flight log only.
//
// Before this becomes real, the following must be resolved:
//   - Which board (A, B, or both) owns which relay channels.
//   - Relay IN lines held in their safe state as the FIRST thing in setup(),
//     with external pull-ups: the Due boots with pins high-Z, and cheap relay
//     boards are commonly active-low, so a boot glitch could otherwise close a
//     relay while the pyro rail is live.
//   - Wiring to the relay NO contacts, so de-energized means no firing path.
//   - Confirm the Due's 3.3V logic actually drives the relay coil reliably.
// ---------------------------------------------------------------------------

enum PyroChannel {
  PYRO_DROGUE_PRIMARY = 0,
  PYRO_DROGUE_BACKUP,
  PYRO_MAIN_PRIMARY,
  PYRO_MAIN_BACKUP,
  PYRO_CHANNEL_COUNT
};

void pyroInit();

// Records the firing event. Does not drive any output.
void pyroFire(PyroChannel ch, float altitudeM, uint32_t nowMs);

bool pyroHasFired(PyroChannel ch);

const char *pyroChannelName(PyroChannel ch);

#endif
