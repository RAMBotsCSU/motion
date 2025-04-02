#ifndef AXIS_H
#define AXIS_H

#include <HardwareSerial.h>
#include <ODriveArduino.h>
#include <Ramp.h>

// Forward declare ODrive to avoid circular dependency.
class ODrive;

class Axis {
    private:
        ODrive& odrive;
        int id;

        rampFloat ramp;
        float offset = 0.0f;
        bool initialized;
        float currentPos = 0.0f;
        float targetPos = 0.0f;
        float startingPos = 0.0f;

    public:
        Axis(ODrive& _odrive, int _id);

        void init(void);
        void fetchOffset(void);
        float getOffset(void);
        int fetchError(void);
        void reset(void);
        void setClosedLoop(void);
        int fetchState(void);

        bool move(float, float);


        float getCurrentPos(void);

    };

#endif  // AXIS_H
