#ifndef AXIS_H
#define AXIS_H

#include <HardwareSerial.h>
#include <ODriveArduino.h>

// Forward declare ODrive to avoid circular dependency.
class ODrive;

class Axis {
    private:
        ODrive& odrive;
        int id;
        int _error;

        float offset = 0.0f;
        float targetPos = 0.0f;
        float _speed = 0.0f;

    public:
        Axis(ODrive& _odrive, int _id);

        void init(void);
        void fetchOffset(void);
        float getOffset(void);
        int fetchError(void);
        int getError(void) { return _error; }
        void reset(void);
        void setClosedLoop(void);
        int fetchState(void);
        void setSpeed(float);

        void move(float);
    };

#endif  // AXIS_H
