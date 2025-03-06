#ifndef LOG_H
#define LOG_H

// Pin 35 is TX
#define SerialMon Serial8

void Log(const char* format, ...);

#endif  // LOG_H
