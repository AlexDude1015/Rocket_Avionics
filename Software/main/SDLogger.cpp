#include "SDLogger.h"
#include "Config.h"
#include <SPI.h>
#include <SD.h>

static bool sdReady = false;

bool sdBegin() {
  sdReady = SD.begin(SD_CS_PIN);
  return sdReady;
}

void sdWriteLine(const String &line) {
  if (!sdReady) return;

  File logFile = SD.open(GROUND_LOG_FILENAME, FILE_WRITE);
  if (logFile) {
    logFile.println(line);
    logFile.close();
  }
}
