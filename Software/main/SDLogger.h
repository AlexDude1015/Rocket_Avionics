#ifndef SDLOGGER_H
#define SDLOGGER_H

#include <Arduino.h>

bool sdBegin();
void sdWriteLine(const String &line);

#endif
