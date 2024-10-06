#ifndef MAIN_H
#define MAIN_H

#include <ODriveArduino.h>

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

//**************remote control****************
extern struct RECEIVE_DATA_STRUCTURE {
    // put your variable definitions here for the data you want to send
    // THIS MUST BE EXACTLY THE SAME ON THE OTHER ARDUINO
    int16_t menuDown;
    int16_t Select;
    int16_t menuUp;
    int16_t toggleBottom;
    int16_t toggleTop;
    int16_t toggle1;
    int16_t toggle2;
    int16_t mode;
    int16_t RLR;
    int16_t RFB;
    int16_t RT;
    int16_t LLR;
    int16_t LFB;
    int16_t LT;
} mydata_remote;

extern unsigned long currentMillis;
extern long previousInterpMillis;
extern int interpFlag;

#endif
