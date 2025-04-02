#include <string>

#include "axis.hpp"
#include "log.hpp"


std::string AXIS_CONFIG[] = {
    "config.startup_closed_loop_control 1",

    "controller.config.pos_gain 60",
    "controller.config.vel_gain 0.1",
    "controller.config.vel_integrator_gain 0.2",
    "controller.config.vel_limit 500",

    "motor.config.pole_pairs 20",
    "motor.config.torque_constant 0.025", // 8.27/90
    "motor.config.current_lim 22.0",
    "motor.config.current_lim_margin 9.0",

    "encoder.config.cpr 16384",
    "encoder.config.mode 257", // Set encoder to SPI mode
};

void Axis::init() {
    // check for errors and attempt to reset them
    for(std::size_t i=0; i<3; ++i) {
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
    for (size_t i = 0; i < sizeof(AXIS_CONFIG) / sizeof(AXIS_CONFIG[0]); ++i) {
        serial.printf("w axis%d.%s\n", id, AXIS_CONFIG[i].c_str());

        String resp = serial.readStringUntil('\n');
        if(resp.length() > 0) { // the odrive will only respond if the command fails
            Log("  w axis%d.%s reponded: %s\n", id, AXIS_CONFIG[i].c_str(), resp.c_str());
        }
    }
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
