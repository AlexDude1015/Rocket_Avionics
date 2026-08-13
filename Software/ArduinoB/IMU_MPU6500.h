#ifndef IMU_MPU6500_H
#define IMU_MPU6500_H

#include <Arduino.h>

// ---------------------------------------------------------------------------
// MPU-6500 6-axis IMU, driven over SPI.
//
// Provides the accelerometer and gyroscope readings the flight code runs on.
// The accelerometer is configured to its widest range (+/-16g) because boost
// on this vehicle peaks around 10-11g; a narrower range would clip and lose
// the boost data entirely.
//
// Note on what an accelerometer actually measures: it senses "specific force",
// which EXCLUDES gravity. Sitting still it reads 1g (the rail pushing up on
// it), and in free fall it reads 0. During unpowered coast it therefore reads
// drag alone. This is why burnout shows up as a sign flip rather than a drop
// to zero.
// ---------------------------------------------------------------------------

// Wakes the sensor, verifies its identity and applies the flight configuration
// (SPI-only mode, +/-2000 dps gyro, +/-16g accelerometer).
// Must be called once before any other function here.
// Returns true on success, or false if the WHO_AM_I register does not answer
// with the expected MPU-6500 value - which normally means a wiring, power or
// chip-select problem rather than a software fault.
bool imuBegin();

// Measures and stores the gyroscope's zero-rate offset by averaging 500
// samples. Blocks for roughly one second, so call it only at startup.
// The vehicle MUST be completely still, since any real rotation during this
// window is baked in as a permanent error.
void imuCalibrate();

// Reads all six axes in a single 14-byte SPI burst, which keeps the
// accelerometer and gyroscope samples from different instants in time.
// Values are raw sensor counts, uncorrected.
//   ax, ay, az - accelerometer counts, written by this function
//   gx, gy, gz - gyroscope counts, written by this function
void imuReadRaw(int16_t &ax, int16_t &ay, int16_t &az,
                int16_t &gx, int16_t &gy, int16_t &gz);

// Subtracts the stored bias from raw gyroscope counts.
//   gx, gy, gz             - raw gyroscope counts to correct
//   gx_out, gy_out, gz_out - bias-corrected results, written by this function
// Returns zeroes only in the sense that an uncalibrated bias is 0, so calling
// this before imuCalibrate() simply passes the raw values through.
void imuGetCorrectedGyro(int16_t gx, int16_t gy, int16_t gz,
                          float &gx_out, float &gy_out, float &gz_out);

// One burst read with both conversions applied, which is the form the flight
// loop wants: accelerometer scaled from counts into g, gyroscope bias-removed.
//   ax_g, ay_g, az_g       - acceleration in g, written by this function
//   gx_out, gy_out, gz_out - bias-corrected gyroscope, written by this function
void imuReadG(float &ax_g, float &ay_g, float &az_g,
              float &gx_out, float &gy_out, float &gz_out);

// Total acceleration magnitude, independent of how the board is oriented.
// Liftoff detection uses this so it does not depend on the orientation
// capture succeeding.
//   ax_g, ay_g, az_g - acceleration components in g
// Returns sqrt(ax^2 + ay^2 + az^2), which reads ~1g whenever the vehicle is
// at rest in any attitude.
float imuAccelMagnitudeG(float ax_g, float ay_g, float az_g);

#endif
