#include <string>

#include "axis.hpp"
#include "odrive.hpp"
#include "config.hpp"
#include "log.hpp"

// axis speeds at 100% speed
const float VEL_LIMIT = 300.0,
            ACCEL_LIMIT = 180.0,
            DECEL_LIMIT = 180.0;


std::string AXIS_CONFIG[] = {
    "config.startup_closed_loop_control 1",

    "controller.config.pos_gain 60",
    "controller.config.vel_gain 0.1",
    "controller.config.vel_integrator_gain 0.2",
    "controller.config.vel_limit 500",

    "motor.config.torque_constant 0.025", // 8.27/90
    "motor.config.current_lim 22.0",
    "motor.config.current_lim_margin 9.0",

    // TRAJECTORY CONTROL
    // Not documented well in 0.5.x, but still exists
    // https://docs.odriverobotics.com/v/latest/manual/control.html#trajectory-control
    "trap_traj.config.vel_limit 300.0",
    "trap_traj.config.accel_limit 1.0",
    "trap_traj.config.decel_limit 1.0",
    "controller.config.inertia 0",

    "controller.config.input_mode 5", // InputMode.TRAP_TRAJ
};


Axis::Axis(ODrive& _odrive, int _id) : odrive(_odrive), id(_id) {}

void Axis::init() {
    // check for errors and attempt to reset them
    int err = 0;
    for(std::size_t i=0; i<3; ++i) {
        err = fetchError();
        Log("  Error %d\n", err);

        if(err == 0) break;

        Log("    Attempting reset...\n");
        reset();

        delay(100);
    }

    if(err > 0) {
        Log("    Failed to reset error\n");
        return;
    }

    // update axis zero pos offsets
    fetchOffset();
    Log("  Offset: %f\n", getOffset());

    // set closed loop
    if(fetchState() != AXIS_STATE_CLOSED_LOOP_CONTROL) setClosedLoop();

    int state = fetchState();
    Log("  Axis state: %d\n", state);

    for (size_t i = 0; i < sizeof(AXIS_CONFIG) / sizeof(AXIS_CONFIG[0]); ++i) {
        String resp = odrive.send("w axis%d.%s", id, AXIS_CONFIG[i].c_str());
        if(resp.length() > 0) { // the odrive will only respond if the command fails
            Log("  w axis%d.%s reponded: %s\n", id, AXIS_CONFIG[i].c_str(), resp.c_str());
        }
    }

    setSpeed(_speed);
}

void Axis::fetchOffset() {
    offset = odrive.send("r axis%d.encoder.pos_estimate\n", id).toFloat();
}

void Axis::reset() {
    odrive.send("w axis%d.controller.error 0", id);
    odrive.send("w axis%d.encoder.error 0", id);
    odrive.send("w axis%d.motor.error 0", id);
    odrive.send("w axis%d.error 0", id);
    odrive.send("sc\n", id);
}

void Axis::setClosedLoop() {
    odrive.send("w axis%d.requested_state %d", id, AXIS_STATE_CLOSED_LOOP_CONTROL);
}

int Axis::fetchState() {
    return odrive.send("r axis%d.current_state", id).toInt();
}

void Axis::move(float pos) {
    if(abs(pos - targetPos) > 0.001) {  // Only start new movement if target actually changed
        targetPos = pos;

        odrive.send("t %d %f", id, targetPos + offset);
    }
}

float Axis::getOffset() {
    return offset;
}

int Axis::fetchError() {
}

void Axis::setSpeed(float speed) {
    if(speed < 0) speed = 0.0;
    else if(speed > 1) speed = 1.0f;

    _speed = speed;

    speed = speed * GLOBAL_SPEED;

    odrive.send("w axis%d.trap_traj.config.accel_limit %0.2f", id, ACCEL_LIMIT * speed);
    odrive.send("w axis%d.trap_traj.config.decel_limit %0.2f", id, DECEL_LIMIT * speed);
}
