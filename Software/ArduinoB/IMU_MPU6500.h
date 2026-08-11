#ifndef IMU_MPU6500_H
#define IMU_MPU6500_H

#include <Arduino.h>

bool imuBegin();
void imuCalibrate();
void imuReadRaw(int16_t &ax, int16_t &ay, int16_t &az,
                int16_t &gx, int16_t &gy, int16_t &gz);
void imuGetCorrectedGyro(int16_t gx, int16_t gy, int16_t gz,
                          float &gx_out, float &gy_out, float &gz_out);

// Convenience wrapper: one burst read, accelerometer scaled to g and gyro
// bias-corrected, as the flight loop wants them.
void imuReadG(float &ax_g, float &ay_g, float &az_g,
              float &gx_out, float &gy_out, float &gz_out);

float imuAccelMagnitudeG(float ax_g, float ay_g, float az_g);

#endif
