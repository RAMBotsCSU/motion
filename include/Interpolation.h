#ifndef INTERPOLATION_H
#define INTERPOLATION_H

#include <Ramp.h>

class Interpolation {
   public:
    rampInt myRamp;
    int interpolationFlag = 0;
    int savedValue;

    int go(int input, int duration) {
        if (input != savedValue) {  // check for new data
            interpolationFlag = 0;
        }
        savedValue = input;  // bookmark the old value

        if (interpolationFlag == 0) {                         // only do it once until the flag is reset
            myRamp.go(input, duration, LINEAR, ONCEFORWARD);  // start interpolation (value to go to, duration)
            interpolationFlag = 1;
        }

        int output = myRamp.update();
        return output;
    }
};  // end of class

#endif
