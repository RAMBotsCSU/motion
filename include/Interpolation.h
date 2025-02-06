#ifndef INTERPOLATION_H
#define INTERPOLATION_H

#include <Ramp.h>

class Interpolation {
   public:
    rampInt myRamp;
    bool interpolationFlag = false;
    float savedValue;

    float go(float input, int duration) {
        if (input != savedValue) {  // check for new data
            interpolationFlag = false;
        }
        savedValue = input;  // bookmark the old value

        // SerialMon.printf("Interpolation go input=%f duration=%d interpolationFlag=%d savedValue=%f\n", input, duration, interpolationFlag, savedValue);

        if (interpolationFlag == false) {                         // only do it once until the flag is reset
            myRamp.go((long)(input * 1000), duration, LINEAR, ONCEFORWARD);  // start interpolation (value to go to, duration)
            interpolationFlag = true;
        }

        float output = (float)myRamp.update() / 1000;

        // SerialMon.printf("Interpolation go output=%f\n", output);

        return output;
    }
};  // end of class

#endif
