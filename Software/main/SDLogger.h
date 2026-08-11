#ifndef SDLOGGER_H
#define SDLOGGER_H

#include <Arduino.h>

// The file is opened once and held open. Re-opening and closing per line (the
// original approach) costs milliseconds per write, which stalls a 20 Hz logging
// loop. Data is flushed on a cadence instead, and closed properly on landing.

bool sdBegin();
bool sdReady();

void sdLogSample(uint32_t tMs, const char *state,
                 float ax_g, float ay_g, float az_g,
                 float gx, float gy, float gz,
                 float altitudeM, float accelMagG, float axialG);

void sdLogEvent(const char *type, const char *detail,
                float altitudeM, uint32_t tMs);

void sdFlush();
void sdClose();

#endif
