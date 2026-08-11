#ifndef ORIENTATION_H
#define ORIENTATION_H

#include <Arduino.h>

// Captures the gravity direction in body frame while the rocket sits vertical
// and still on the rail, so burnout detection can project onto the true flight
// axis. Storing the whole vector rather than picking the dominant axis absorbs
// rail tilt and IMU mounting misalignment in one step, and the sign falls out
// automatically (positive = nose direction).
//
// This must run at ARMING, not at boot - at boot the bay may be lying on a
// bench in any orientation.

// Returns false (and leaves the stored vector untouched) if the reading does
// not look like a vehicle standing still under 1g.
bool orientationCapture(float ax_g, float ay_g, float az_g);

bool orientationValid();

// Signed acceleration along the flight axis. Meaningless until capture succeeds.
float orientationAxial(float ax_g, float ay_g, float az_g);

void orientationGetUp(float &x, float &y, float &z);

#endif
