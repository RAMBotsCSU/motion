#ifndef ODRIVE_H
#define ODRIVE_H

#include "axis.hpp"

#include <HardwareSerial.h>
#include <ODriveArduino.h>


class ODrive {
private:
    HardwareSerial &serial;
    ODriveArduino odrive;

public:
    ODrive(HardwareSerial& _serial) : serial(_serial), odrive(_serial), axis0(_serial, odrive, 0), axis1(serial, odrive, 1) {}
    bool init(void);

    Axis axis0;
    Axis axis1;
};

#endif  // ODRIVE_H
