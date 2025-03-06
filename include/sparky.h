#ifndef SPARKY_H
#define SPARKY_H

#include <Adafruit_MPU6050.h>

#include "odrive.h"
#include "leg.h"
#include "kinematics.h"


class Sparky {
public:
    Sparky();
    void setup();
    void update();


private:
    bool enabled = false;
    int requestedMode = 0;
    unsigned long lastTick;
    unsigned long remoteLastSeen;

    int LFB, LLR, LT, RFB, RLR, RT;

    Adafruit_MPU6050 mpu;

    ODrive odrive[6];
    Leg leg[4];
    Kinematics kinematics;
};

#endif  // SPARKY_H
