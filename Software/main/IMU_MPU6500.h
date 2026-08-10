#ifndef IMU_MPU6500_H
#define IMU_MPU6500_H

#include <Arduino.h>

bool imuBegin();
void imuCalibrate();
void imuReadRaw(int16_t &ax, int16_t &ay, int16_t &az,
                int16_t &gx, int16_t &gy, int16_t &gz);
void imuGetCorrectedGyro(int16_t gx, int16_t gy, int16_t gz,
                          float &gx_out, float &gy_out, float &gz_out);

#endif
