#include "SDLogger.h"
#include "Config.h"
#include <SPI.h>
#include <SD.h>

static bool sdOpen = false;   // true only when logFile is genuinely usable
static File logFile;          // held open for the whole flight

// See SDLogger.h.
// Both steps can fail independently - the card may mount but the file still
// fail to open - so each is checked separately.
bool sdBegin() {
  sdOpen = false;

  if (!SD.begin(SD_CS_PIN)) return false;

  logFile = SD.open(GROUND_LOG_FILENAME, FILE_WRITE);
  if (!logFile) return false;

  sdOpen = true;
  logFile.println("t_ms,rec,state,ax_g,ay_g,az_g,gx,gy,gz,alt_m,mag_g,axial_g,detail");
  logFile.flush();   // get the header onto the card immediately
  return true;
}

// See SDLogger.h.
bool sdReady() {
  return sdOpen;
}

// See SDLogger.h.
// Formatting into a fixed buffer rather than building an Arduino String avoids
// repeatedly allocating and freeing memory, which at 20 Hz over a long flight
// would gradually fragment the heap.
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
  logFile.println(buf);   // buffered; sdFlush() commits it
}

// See SDLogger.h.
// The empty commas hold the columns a data row would fill, so every line has
// the same shape and spreadsheets read the file without complaint.
void sdLogEvent(const char *type, const char *detail,
                float altitudeM, uint32_t tMs) {
  if (!sdOpen) return;

  char buf[192];
  snprintf(buf, sizeof(buf),
           "%lu,E,%s,,,,,,,%.2f,,,%s",
           (unsigned long)tMs, type, altitudeM, detail);
  logFile.println(buf);
  logFile.flush();   // events are rare and matter most - never leave buffered
}

// See SDLogger.h.
void sdFlush() {
  if (sdOpen) logFile.flush();
}

// See SDLogger.h.
// Clearing sdOpen means any later logging call quietly does nothing rather
// than writing to a closed file.
void sdClose() {
  if (!sdOpen) return;
  logFile.flush();
  logFile.close();
  sdOpen = false;
}
