#ifndef SDLOGGER_H
#define SDLOGGER_H

#include <Arduino.h>

// ---------------------------------------------------------------------------
// Flight data recording to the microSD card, written as CSV.
//
// The flight lasts about five minutes and cannot be repeated, so the log is
// the only record of what actually happened. Two design choices follow from
// that:
//
// THE FILE IS HELD OPEN. Opening and closing it for every line - the obvious
// approach - costs milliseconds each time, which would stall a loop trying to
// run at 20 Hz. Instead it is opened once and flushed periodically.
//
// FLUSHING MATTERS. Data written to an open file sits in a buffer until
// flushed, and anything still buffered is lost if power is cut. Regular
// flushes bound how much can be lost; sdClose() on landing makes the file
// safe.
//
// Two kinds of row share the file, distinguished by the "rec" column:
//   D - a periodic data sample
//   E - a one-off event, such as a state change or a charge firing
// ---------------------------------------------------------------------------

// Mounts the card, opens the log file and writes the CSV header row.
// Returns true on success, false if the card is missing, unformatted or the
// file cannot be opened.
//
// Note: the file is opened in append mode, so repeated power cycles add to the
// same file rather than replacing it.
bool sdBegin();

// Returns true if the log file is open and writable.
bool sdReady();

// Writes one periodic data row. Does nothing if the log is not open, so
// callers never need to check first.
//   tMs        - millis() timestamp for this sample
//   state      - flight phase name, from flightStateName()
//   ax_g/ay_g/az_g - acceleration components in g
//   gx/gy/gz   - bias-corrected gyroscope readings
//   altitudeM  - height above the pad, in metres
//   accelMagG  - total acceleration magnitude in g
//   axialG     - signed acceleration along the flight axis, in g
void sdLogSample(uint32_t tMs, const char *state,
                 float ax_g, float ay_g, float az_g,
                 float gx, float gy, float gz,
                 float altitudeM, float accelMagG, float axialG);

// Writes one event row and flushes immediately, since events are rare, cheap
// to write and the most important thing in the log to not lose.
//   type      - short event category, such as "STATE" or "PYRO_STUB"
//   detail    - free-text detail, such as the state or channel name
//   altitudeM - altitude when the event occurred, in metres
//   tMs       - millis() timestamp
void sdLogEvent(const char *type, const char *detail,
                float altitudeM, uint32_t tMs);

// Pushes buffered data out to the card. Called on a timer during flight to
// limit how much could be lost to a sudden power failure.
void sdFlush();

// Flushes and closes the file properly. Called once landing is detected, which
// is the main reason landing detection exists at all: a file left open when
// the battery is disconnected can lose its contents.
void sdClose();

#endif
