#include "Orientation.h"
#include "Config.h"
#include <math.h>

// The captured "up" direction as a unit vector in the IMU's own frame.
static float upX = 0, upY = 0, upZ = 0;
static bool valid = false;

// See Orientation.h.
// Dividing each component by the total length normalises the vector, so later
// projections return acceleration in g rather than a scaled-up number.
bool orientationCapture(float ax_g, float ay_g, float az_g) {
  float mag = sqrt(ax_g * ax_g + ay_g * ay_g + az_g * az_g);

  // Guard: a stationary vehicle reads almost exactly 1g. Anything else means
  // it is being handled, is vibrating, or the sensor is misbehaving - none of
  // which give a trustworthy reference direction.
  if (mag < (1.0f - ARM_ACCEL_TOL_G) || mag > (1.0f + ARM_ACCEL_TOL_G)) {
    return false;
  }

  upX = ax_g / mag;
  upY = ay_g / mag;
  upZ = az_g / mag;
  valid = true;
  return true;
}

// See Orientation.h.
bool orientationValid() {
  return valid;
}

// See Orientation.h.
// This is a dot product. Because the stored vector has unit length, it returns
// how much of the acceleration points along the rocket's axis, keeping the
// sign: positive toward the nose, negative toward the tail.
float orientationAxial(float ax_g, float ay_g, float az_g) {
  return ax_g * upX + ay_g * upY + az_g * upZ;
}

// See Orientation.h.
void orientationGetUp(float &x, float &y, float &z) {
  x = upX;
  y = upY;
  z = upZ;
}
