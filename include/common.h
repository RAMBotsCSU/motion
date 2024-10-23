#ifndef MAIN_H
#define MAIN_H

#include <ODriveArduino.h>

#define SerialMon Serial8

// Printing with stream operator
template <class T>
inline Print& operator<<(Print& obj, T arg) {
    obj.print(arg);
    return obj;
}
template <>
inline Print& operator<<(Print& obj, float arg) {
    obj.print(arg, 4);
    return obj;
}

extern unsigned long currentMillis;
extern long previousInterpMillis;
extern int interpFlag;

#endif
