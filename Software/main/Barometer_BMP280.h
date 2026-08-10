#ifndef BAROMETER_BMP280_H
#define BAROMETER_BMP280_H

#include <Arduino.h>

bool baroBegin();
void baroCaptureGroundReference();
float baroReadAltitude();
float baroReadPressure();

#endif
