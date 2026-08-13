#ifndef PYRO_H
#define PYRO_H

#include <Arduino.h>

// ---------------------------------------------------------------------------
// Parachute deployment charges.
//
// SAFETY: THIS MODULE DOES NOT ACTUATE ANYTHING.
//
// Whether Arduino B drives the relay board at all is still an open hardware
// decision, so no relay pins are defined here and nothing can be energised.
// pyroFire() records the event to serial and the flight log only. This keeps
// the flight logic complete and testable on the bench without any possibility
// of setting off a charge.
//
// Background: there are two parachutes, a drogue for the fast initial descent
// and a main for the final approach. Each has two charges, a primary and a
// backup - four in total, matching the relay board's four channels. All four
// fire unconditionally on phase-gated timers, with no attempt to detect
// whether a charge worked, so that no live black powder is left in the rocket
// when it lands.
//
// Before this becomes real, the following must be resolved:
//   - Which board (A, B, or both) owns which relay channels.
//   - Relay IN lines held in their safe state as the FIRST thing in setup(),
//     with external pull-ups: the Due boots with pins high-impedance, and
//     cheap relay boards are commonly active-low, so a boot glitch could
//     otherwise close a relay while the firing rail is live.
//   - Wiring to the relay NO contacts, so de-energised means no firing path.
//   - Confirm the Due's 3.3V logic actually drives the relay coil reliably.
// ---------------------------------------------------------------------------

enum PyroChannel {
  PYRO_DROGUE_PRIMARY = 0,
  PYRO_DROGUE_BACKUP,
  PYRO_MAIN_PRIMARY,
  PYRO_MAIN_BACKUP,
  PYRO_CHANNEL_COUNT     // count of channels; not a channel itself
};

// Clears the fired-state record for all channels. Call once at startup.
void pyroInit();

// Records that a channel would have fired, and marks it as spent so repeated
// calls are ignored - the state machine calls this every loop once its
// trigger condition is met, and only the first call should count.
//   ch        - which charge
//   altitudeM - altitude at firing, in metres, for the log
//   nowMs     - millis() timestamp, for the log
void pyroFire(PyroChannel ch, float altitudeM, uint32_t nowMs);

// Returns true if the given channel has already been fired.
//   ch - channel to query
bool pyroHasFired(PyroChannel ch);

// Converts a channel to a readable name for serial output and the log.
//   ch - channel to name
// Returns a static string, or "UNKNOWN" for an invalid value.
const char *pyroChannelName(PyroChannel ch);

#endif
