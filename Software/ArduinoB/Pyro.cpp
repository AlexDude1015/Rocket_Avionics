#include "Pyro.h"
#include "SDLogger.h"

static bool fired[PYRO_CHANNEL_COUNT];

void pyroInit() {
  for (int i = 0; i < PYRO_CHANNEL_COUNT; i++) fired[i] = false;
}

const char *pyroChannelName(PyroChannel ch) {
  switch (ch) {
    case PYRO_DROGUE_PRIMARY: return "DROGUE_PRIMARY";
    case PYRO_DROGUE_BACKUP:  return "DROGUE_BACKUP";
    case PYRO_MAIN_PRIMARY:   return "MAIN_PRIMARY";
    case PYRO_MAIN_BACKUP:    return "MAIN_BACKUP";
    default:                  return "UNKNOWN";
  }
}

void pyroFire(PyroChannel ch, float altitudeM, uint32_t nowMs) {
  if (ch >= PYRO_CHANNEL_COUNT || fired[ch]) return;
  fired[ch] = true;

  // No output is driven - see the safety notes in Pyro.h.
  Serial.print("[PYRO-STUB] would fire ");
  Serial.print(pyroChannelName(ch));
  Serial.print(" at t=");
  Serial.print(nowMs);
  Serial.print("ms alt=");
  Serial.print(altitudeM);
  Serial.println("m");

  sdLogEvent("PYRO_STUB", pyroChannelName(ch), altitudeM, nowMs);
}

bool pyroHasFired(PyroChannel ch) {
  if (ch >= PYRO_CHANNEL_COUNT) return false;
  return fired[ch];
}
