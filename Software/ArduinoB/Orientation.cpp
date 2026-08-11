#include "Orientation.h"
#include "Config.h"
#include <math.h>

static float upX = 0, upY = 0, upZ = 0;
static bool valid = false;

bool orientationCapture(float ax_g, float ay_g, float az_g) {
  float mag = sqrt(ax_g * ax_g + ay_g * ay_g + az_g * az_g);

  // A bad up-vector silently corrupts burnout detection, so refuse anything
  // that is not clearly a stationary vehicle sitting under gravity.
  if (mag < (1.0f - ARM_ACCEL_TOL_G) || mag > (1.0f + ARM_ACCEL_TOL_G)) {
    return false;
  }

  upX = ax_g / mag;
  upY = ay_g / mag;
  upZ = az_g / mag;
  valid = true;
  return true;
}

bool orientationValid() {
  return valid;
}

float orientationAxial(float ax_g, float ay_g, float az_g) {
  return ax_g * upX + ay_g * upY + az_g * upZ;
}

void orientationGetUp(float &x, float &y, float &z) {
  x = upX;
  y = upY;
  z = upZ;
}
