#include "SDLogger.h"
#include "Config.h"
#include <SPI.h>
#include <SD.h>

static bool sdOpen = false;
static File logFile;

bool sdBegin() {
  sdOpen = false;

  if (!SD.begin(SD_CS_PIN)) return false;

  logFile = SD.open(GROUND_LOG_FILENAME, FILE_WRITE);
  if (!logFile) return false;

  sdOpen = true;
  logFile.println("t_ms,rec,state,ax_g,ay_g,az_g,gx,gy,gz,alt_m,mag_g,axial_g,detail");
  logFile.flush();
  return true;
}

bool sdReady() {
  return sdOpen;
}

void sdLogSample(uint32_t tMs, const char *state,
                 float ax_g, float ay_g, float az_g,
                 float gx, float gy, float gz,
                 float altitudeM, float accelMagG, float axialG) {
  if (!sdOpen) return;

  char buf[192];
  snprintf(buf, sizeof(buf),
           "%lu,D,%s,%.3f,%.3f,%.3f,%.1f,%.1f,%.1f,%.2f,%.3f,%.3f,",
           (unsigned long)tMs, state,
           ax_g, ay_g, az_g, gx, gy, gz,
           altitudeM, accelMagG, axialG);
  logFile.println(buf);
}

void sdLogEvent(const char *type, const char *detail,
                float altitudeM, uint32_t tMs) {
  if (!sdOpen) return;

  char buf[192];
  snprintf(buf, sizeof(buf),
           "%lu,E,%s,,,,,,,%.2f,,,%s",
           (unsigned long)tMs, type, altitudeM, detail);
  logFile.println(buf);
  logFile.flush();  // events are cheap and worth never losing
}

void sdFlush() {
  if (sdOpen) logFile.flush();
}

void sdClose() {
  if (!sdOpen) return;
  logFile.flush();
  logFile.close();
  sdOpen = false;
}
