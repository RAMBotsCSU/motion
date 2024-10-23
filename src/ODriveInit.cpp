#include <ODriveArduino.h>

#include "common.h"
#include "kinematics.h"


// ODrive
ODriveArduino odrive1 = ODriveArduino(Serial1);
ODriveArduino odrive2 = ODriveArduino(Serial2);
ODriveArduino odrive3 = ODriveArduino(Serial3);
ODriveArduino odrive4 = ODriveArduino(Serial4);
ODriveArduino odrive5 = ODriveArduino(Serial5);
ODriveArduino odrive6 = ODriveArduino(Serial6);


float offsetShoulder = 0.680279;
float offsetKnee = -1.498286;
float offsetHip = 0.000006;

// ODrive offsets from power up
// ratio is 10:1 so 1 'turn' is 36'.
// offset<odrive><axis>
float offSet20 = offsetKnee;    // knee - right front
float offSet30 = offsetKnee;   // knee - right rear
float offSet50 = offsetKnee;   // knee - left front
float offSet60 = offsetKnee;    // knee - left rear

float offSet21 = offsetShoulder;    // shoulder - right front
float offSet31 = offsetShoulder;    // shoulder - right rear
float offSet51 = offsetShoulder;    // shoulder - left front
float offSet61 = offsetShoulder;   // shoulder - left rear

float offSet10 = offsetHip;    // hips - right front
float offSet11 = offsetHip;     // hips - right back
float offSet40 = offsetHip;    // hips - left front
float offSet41 = offsetHip;    // hips - left back

// init hips

int requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL;
float current_lim = 20.0;

void InitODrive(ODriveArduino &odrive, HardwareSerial &serial) {
    for (int axis = 0; axis < 2; ++axis) {
        // Serial1 << "w axis" << axis << ".controller.config.vel_limit " << 6000.0f << '\n';          // *** Velocity limits should be set to infinite through the ODrive tool, this stops the motors disarming under certain situations ***

        // reset errors
        serial.printf("w axis%d.error 0\n", axis);
        serial.printf("w axis%d.motor.error 0\n", axis);
        serial.printf("w axis%d.encoder.error 0\n", axis);
        serial.printf("w axis%d.controller.error 0\n", axis);

        serial << "w axis" << axis << ".motor.config.current_lim " << current_lim << '\n';

        bool temp = odrive.runState(axis, requested_state, false);  // don't wait

        SerialMon.printf("Axis %d requesting state %d - %d\n", axis, requested_state, temp);
    }
}

void OdriveInit1() {

    SerialMon.println("ODrive 1");
    InitODrive(odrive1, Serial1);

    SerialMon.println("ODrive 2");
    InitODrive(odrive2, Serial2);

    SerialMon.println("ODrive 3");
    InitODrive(odrive3, Serial3);

    SerialMon.println("ODrive 4");
    InitODrive(odrive4, Serial4);

    SerialMon.println("ODrive 5");
    InitODrive(odrive5, Serial5);

    SerialMon.println("ODrive 6");
    InitODrive(odrive6, Serial6);

    SerialMon.println("ODRIVE INIT COMPLETE");
}

void modifyGains() {  // this function turns up the gains when it is executed (menu option 4 via the remote)

    SerialMon.println("modfy gains");

    float posGainKnee = 15.0;
    float posGainHips = 70.0;
    float posGainShoulder = 15.0;
    float velGain = 0.1;
    float integrator = 0.1;

    Serial1 << "w axis" << 0 << ".controller.config.pos_gain " << posGainHips << '\n';
    Serial1 << "w axis" << 1 << ".controller.config.pos_gain " << posGainHips << '\n';

    Serial2 << "w axis" << 0 << ".controller.config.pos_gain " << posGainKnee << '\n';
    Serial2 << "w axis" << 1 << ".controller.config.pos_gain " << posGainShoulder << '\n';

    Serial3 << "w axis" << 0 << ".controller.config.pos_gain " << posGainKnee << '\n';
    Serial3 << "w axis" << 1 << ".controller.config.pos_gain " << posGainShoulder << '\n';

    Serial4 << "w axis" << 0 << ".controller.config.pos_gain " << posGainHips << '\n';
    Serial4 << "w axis" << 1 << ".controller.config.pos_gain " << posGainHips << '\n';

    Serial5 << "w axis" << 0 << ".controller.config.pos_gain " << posGainKnee << '\n';
    Serial5 << "w axis" << 1 << ".controller.config.pos_gain " << posGainShoulder << '\n';

    Serial6 << "w axis" << 0 << ".controller.config.pos_gain " << posGainKnee << '\n';
    Serial6 << "w axis" << 1 << ".controller.config.pos_gain " << posGainShoulder << '\n';

    // ******

    Serial1 << "w axis" << 0 << ".controller.config.vel_gain " << velGain << '\n';
    Serial1 << "w axis" << 1 << ".controller.config.vel_gain " << velGain << '\n';

    Serial2 << "w axis" << 0 << ".controller.config.vel_gain " << velGain << '\n';
    Serial2 << "w axis" << 1 << ".controller.config.vel_gain " << velGain << '\n';

    Serial3 << "w axis" << 0 << ".controller.config.vel_gain " << velGain << '\n';
    Serial3 << "w axis" << 1 << ".controller.config.vel_gain " << velGain << '\n';

    Serial4 << "w axis" << 0 << ".controller.config.vel_gain " << velGain << '\n';
    Serial4 << "w axis" << 1 << ".controller.config.vel_gain " << velGain << '\n';

    Serial5 << "w axis" << 0 << ".controller.config.vel_gain " << velGain << '\n';
    Serial5 << "w axis" << 1 << ".controller.config.vel_gain " << velGain << '\n';

    Serial6 << "w axis" << 0 << ".controller.config.vel_gain " << velGain << '\n';
    Serial6 << "w axis" << 1 << ".controller.config.vel_gain " << velGain << '\n';

    // ******

    Serial1 << "w axis" << 0 << ".controller.config.vel_integrator_gain " << integrator << '\n';
    Serial1 << "w axis" << 1 << ".controller.config.vel_integrator_gain " << integrator << '\n';

    Serial2 << "w axis" << 0 << ".controller.config.vel_integrator_gain " << integrator << '\n';
    Serial2 << "w axis" << 1 << ".controller.config.vel_integrator_gain " << integrator << '\n';

    Serial3 << "w axis" << 0 << ".controller.config.vel_integrator_gain " << integrator << '\n';
    Serial3 << "w axis" << 1 << ".controller.config.vel_integrator_gain " << integrator << '\n';

    Serial4 << "w axis" << 0 << ".controller.config.vel_integrator_gain " << integrator << '\n';
    Serial4 << "w axis" << 1 << ".controller.config.vel_integrator_gain " << integrator << '\n';

    Serial5 << "w axis" << 0 << ".controller.config.vel_integrator_gain " << integrator << '\n';
    Serial5 << "w axis" << 1 << ".controller.config.vel_integrator_gain " << integrator << '\n';

    Serial6 << "w axis" << 0 << ".controller.config.vel_integrator_gain " << integrator << '\n';
    Serial6 << "w axis" << 1 << ".controller.config.vel_integrator_gain " << integrator << '\n';
}

void applyOffsets1() {
    // apply initial offsets to ODrives for hips

    odrive1.setPosition(0, offSet10);  // hips - right front
    odrive1.setPosition(1, offSet11);  // hips - right rear
    odrive4.setPosition(0, offSet40);  // hips - left front
    odrive4.setPosition(1, offSet41);  // hips - left rear
}

void applyOffsets2() {
    // apply initial offsets to ODrives for shoulders

    odrive2.setPosition(1, offSet21);  // shoulder - right front
    odrive3.setPosition(1, offSet31);  // shoulder - right rear
    odrive5.setPosition(1, offSet51);  // shoulder - left front
    odrive6.setPosition(1, offSet61);  // shoulder - left rear
    odrive2.setPosition(0, offSet20);  // knee - right front
    odrive3.setPosition(0, offSet30);  // knee - right rear
    odrive5.setPosition(0, offSet50);  // knee - left front
    odrive6.setPosition(0, offSet60);  // knee - left front
}

void driveJoints(int joint, float pos) {
    SerialMon.printf("[DRIVEJOINTS] %d to %f\n", joint, pos);
    // takes into account the original setup offsets for motor postions, and also turns around directions so they are consistent
    // also constrains the motion limts for each joint

    pos = constrain(pos, -2.5, 2.5);

    // knees

    if (joint == 20) {
        odrive2.setPosition(0, pos + offSet20);  // knee - right front
    } else if (joint == 30) {
        odrive3.setPosition(0, (pos * -1) + offSet30);  // knee - right back
    } else if (joint == 50) {
        odrive5.setPosition(0, (pos * -1) + offSet50);  // knee - left front
    } else if (joint == 60) {
        odrive6.setPosition(0, pos + offSet60);  // knee - left back
    }

    // shoulder

    else if (joint == 21) {
        odrive2.setPosition(1, (pos * -1) + offSet21);  // shoulder - right front
    } else if (joint == 31) {
        odrive3.setPosition(1, pos + offSet31);  // shoulder - right rear
    } else if (joint == 51) {
        odrive5.setPosition(1, pos + offSet51);  // shoulder - left front
    } else if (joint == 61) {
        odrive6.setPosition(1, (pos * -1) + offSet61);  // shoulder - left rear
    }

    // hips
    else if (joint == 10) {
        odrive1.setPosition(0, pos + offSet10);  // hips - right front
    } else if (joint == 11) {
        odrive1.setPosition(1, (pos * -1) + offSet11);  // hips - right rear
    } else if (joint == 40) {
        odrive4.setPosition(0, pos + offSet40);  // hips - left front
    } else if (joint == 41) {
        odrive4.setPosition(1, (pos * -1) + offSet41);  // hips - left rear
    }
}
