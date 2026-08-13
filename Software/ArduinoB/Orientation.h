#ifndef ORIENTATION_H
#define ORIENTATION_H

#include <Arduino.h>

// ---------------------------------------------------------------------------
// Works out which way is "up" for this particular installation, so the flight
// code can tell forward acceleration from backward.
//
// The problem it solves: burnout is detected by acceleration along the rocket's
// long axis changing sign. But the IMU could be mounted facing any direction
// inside the bay, so no single axis (X, Y or Z) can be assumed to be that one.
//
// The trick: while the rocket stands still on the rail, the only thing the
// accelerometer feels is gravity. So the direction it reports IS the vertical
// axis, expressed in whatever frame the board happens to be mounted in.
// Recording that whole direction, rather than picking the single largest axis,
// also absorbs launch rail tilt and any mounting misalignment, and gives the
// correct sign automatically.
//
// TIMING MATTERS: this must run at arming, when the rocket is upright on the
// rail. Running it at boot would capture whatever way the bay happened to be
// lying on the bench.
// ---------------------------------------------------------------------------

// Records the current acceleration direction as "up", normalised to unit length.
//   ax_g, ay_g, az_g - current acceleration in g, vehicle stationary and upright
// Returns false and changes nothing if the total does not look like a vehicle
// sitting still under gravity (magnitude too far from 1g), which means the
// reading cannot be trusted. Arming refuses to proceed in that case, because a
// wrong up-vector would silently corrupt burnout detection in flight.
bool orientationCapture(float ax_g, float ay_g, float az_g);

// Returns true once a valid up direction has been captured.
bool orientationValid();

// Projects an acceleration reading onto the stored flight axis.
//   ax_g, ay_g, az_g - acceleration components in g
// Returns signed acceleration along the rocket's length, positive toward the
// nose. Roughly +1g on the pad, strongly positive under thrust, and negative
// during coast when drag pushes backward. Meaningless until orientationCapture()
// has succeeded.
float orientationAxial(float ax_g, float ay_g, float az_g);

// Reads back the stored up direction, mainly for logging and diagnostics.
//   x, y, z - unit vector components, written by this function
void orientationGetUp(float &x, float &y, float &z);

#endif
