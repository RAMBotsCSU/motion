#ifndef SPARKY_H
#define SPARKY_H

#include <Adafruit_MPU6050.h>

#include "odrive.hpp"
#include "leg.hpp"
#include "kinematics.hpp"


enum MotionMode {
    HOME    = 0,
    PUSH_UP = 4,
    DANCE   = 5,
    WALK    = 6,
};


class Sparky {
public:
    Sparky();
    void setup();
    void update();
    void setSpeed(float);


private:
    bool _enabled = false;
    int requestedMode = 0;
    int currentMode = 0;
    unsigned long lastTick;
    unsigned long remoteLastSeen;

    int LFB, LLR, LT, RFB, RLR, RT;
    bool DPAD_U, DPAD_D, DPAD_L, DPAD_R, TRIANGLE, CROSS, SQUARE, CIRCLE;

    Adafruit_MPU6050 mpu;

    ODrive odrive[6];
    Leg leg[4];
    Kinematics kinematics;
};

#endif  // SPARKY_H
