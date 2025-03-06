#include <Arduino.h>

#include "log.hpp"
#include "config.hpp"

void Log(const char* format, ...) {
    // sending serial data takes time, if we run the debug logger
    // under normal operation it will cause the main loop to desync
    if(!DEBUG) return; // ensure no logging if not debugging
    va_list args;
    va_start(args, format);
    vprintf(format, args);
    SerialMon.vprintf(format, args);
    va_end(args);
}
