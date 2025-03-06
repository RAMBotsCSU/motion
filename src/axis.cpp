#include "axis.h"
#include "log.h"

Axis::Axis(HardwareSerial& _serial, ODriveArduino& _odrive, int _id) : serial(_serial), odrive(_odrive), id(_id), ramp(0) {}

void Axis::init() {
    // check for errors and attempt to reset them
    for(int i=0; i<3; ++i) {
        int err = getError();
        Log("Axis error %d\n", err);

        if(err == 0) break;

        Log("Attempting reset...\n");
        reset();

        delay(100);
    }

    // update axis zero pos offsets
    fetchOffset();
    Log("Offset: %f\n", getOffset());

    // set closed loop
    if(fetchState() != AXIS_STATE_CLOSED_LOOP_CONTROL) setClosedLoop();

    int state = fetchState();
    Log("Axis state: %d\n", state);
}

void Axis::fetchOffset() {
    offset = odrive.getPosition(id);
}

void Axis::reset() {
    serial.printf("w axis%d.controller.error 0\n", id);
    serial.printf("w axis%d.encoder.error 0\n", id);
    serial.printf("w axis%d.motor.error 0\n", id);
    serial.printf("w axis%d.error 0\n", id);
    serial.printf("sc\n", id);
}

void Axis::setClosedLoop() {
    odrive.runState(id, AXIS_STATE_CLOSED_LOOP_CONTROL, false);
}

int Axis::fetchState() {
    serial.printf("r axis%d.current_state\n", id);
    return serial.readStringUntil('\n').toInt();
}

bool Axis::move(float pos, float duration) {
    if(abs(pos - targetPos) > 0.0001) {  // Only start new movement if target actually changed
        if(!ramp.isRunning()) {
            targetPos = pos;

            if(duration == 0) {
                float max_velocity = 2.0;
                float distance = abs(currentPos - pos);
                duration = distance / max_velocity;
            }

            startingPos = currentPos;  // Start from current position for smooth transition
            ramp.go(pos - startingPos, duration, LINEAR, ONCEFORWARD);
            Log("ramp.go from %f to %f over %f seconds\n", startingPos, pos, duration);
        }
    }

    if(ramp.isRunning()) {
        currentPos = startingPos + ramp.update();
        odrive.setPosition(id, currentPos + offset);
        Log("setPosition %f (target: %f)\n", currentPos, targetPos);
    }

    return !ramp.isRunning();
}

float Axis::getCurrentPos() {
    return currentPos;
}

float Axis::getOffset() {
    return offset;
}

int Axis::getError() {
    serial.printf("r axis%d.error\n", id);
    return serial.readStringUntil('\n').toInt();
}
