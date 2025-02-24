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

// kinematics 0 position offsets
float offsetShoulder = 0.744587;
float offsetKnee = -1.489174;
float offsetHip = 0.000006;

// odrive startup 0 position offsets
float startingPositions[6][2];

int requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL;
float current_lim = 20.0;

void InitODrive(ODriveArduino &odrive, HardwareSerial &serial, int odriveIndex) {
    for (int axis = 0; axis < 2; ++axis) {
        serial.setTimeout(100);

        // check error
        serial.printf("r axis%d.error\n", axis);
        int err = serial.readStringUntil('\n').toInt();
        SerialMon.printf("Axis %d error: %d\n", axis, err);

        // reset errors
        serial.printf("w axis%d.controller.error 0\n", axis);
        serial.printf("w axis%d.encoder.error 0\n", axis);
        serial.printf("w axis%d.motor.error 0\n", axis);
        serial.printf("w axis%d.error 0\n", axis);
        serial.printf("sc\n", axis);

        // check error
        serial.printf("r axis%d.error\n", axis);
        err = serial.readStringUntil('\n').toInt();
        SerialMon.printf("Axis %d error after reset: %d\n", axis, err);

        // config
        serial.printf("w axis%d.motor.config.current_lim %f\n", axis, current_lim);

        // closed loop
        odrive.runState(axis, requested_state, false);
        serial.printf("r axis%d.current_state\n", axis);
        int state = serial.readStringUntil('\n').toInt();
        SerialMon.printf("Axis %d requesting state %d - %d\n", axis, requested_state, state);

        // Get starting position
        float pos = odrive.getPosition(axis);
        startingPositions[odriveIndex][axis] = pos;
        SerialMon.printf("Axis %d position: %f\n", axis, pos);
    }
}

void OdriveInit1() {

    SerialMon.println("ODrive 1");
    InitODrive(odrive1, Serial1, 0);

    SerialMon.println("ODrive 2");
    InitODrive(odrive2, Serial2, 1);

    SerialMon.println("ODrive 3");
    InitODrive(odrive3, Serial3, 2);

    SerialMon.println("ODrive 4");
    InitODrive(odrive4, Serial4, 3);

    SerialMon.println("ODrive 5");
    InitODrive(odrive5, Serial5, 4);

    SerialMon.println("ODrive 6");
    InitODrive(odrive6, Serial6, 5);

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

void driveJoints(int joint, float pos) {
    // SerialMon.printf("[DRIVEJOINTS] %d to %f\n", joint, pos);
    // takes into account the original setup offsets for motor postions, and also turns around directions so they are consistent
    // also constrains the motion limts for each joint

    pos = constrain(pos, -2.5, 2.5);

    // return;

    // knees

    if (joint == 20) {
        odrive2.setPosition(0, pos + offsetKnee + startingPositions[1][0]);  // knee - right front
        // SerialMon.printf("Set position for joint %d: axis 0, position %f, pos=%f\n", joint, pos + offsetKnee + startingPositions[1][0], pos);
        // currentPos = odrive2.getPosition(0);
    } else if (joint == 30) {
        odrive3.setPosition(0, pos + offsetKnee + startingPositions[2][0]);  // knee - right back
        // SerialMon.printf("Set position for joint %d: axis 0, position %f, pos=%f\n", joint, pos + offsetKnee + startingPositions[2][0], pos);
        // currentPos = odrive3.getPosition(0);
    } else if (joint == 50) {
        odrive5.setPosition(0, -pos - offsetKnee + startingPositions[4][0]);  // knee - left front
        // SerialMon.printf("Set position for joint %d: axis 0, position %f, pos=%f\n", joint, pos + offsetKnee + startingPositions[4][0], pos);
        // SerialMon.printf("Set position for joint %d: axis 0, position %f, pos=%f\n", joint, -pos - offsetKnee + startingPositions[4][0], pos);
        // currentPos = odrive5.getPosition(0);
    } else if (joint == 60) {
        odrive6.setPosition(0, -pos - offsetKnee + startingPositions[5][0]);  // knee - left back
        // SerialMon.printf("Set position for joint %d: axis 0, position %f, pos=%f\n", joint, pos + offsetKnee + startingPositions[5][0], pos);
        // currentPos = odrive6.getPosition(0);
    }

    // shoulder

    else if (joint == 21) {
        odrive2.setPosition(1, pos + offsetShoulder + startingPositions[1][1]);  // shoulder - right front
        // SerialMon.printf("Set position for joint %d: axis 1, position %f, pos=%f\n", joint, pos + offsetShoulder + startingPositions[1][1], pos);
        // currentPos = odrive2.getPosition(1);
    } else if (joint == 31) {
        odrive3.setPosition(1, pos + offsetShoulder + startingPositions[2][1]);  // shoulder - right rear
        // SerialMon.printf("Set position for joint %d: axis 1, position %f, pos=%f\n", joint, pos + offsetShoulder + startingPositions[2][1], pos);
        // currentPos = odrive3.getPosition(1);
    } else if (joint == 51) {
        odrive5.setPosition(1, -pos - offsetShoulder + startingPositions[4][1]);  // shoulder - left front
        // SerialMon.printf("Set position for joint %d: axis 1, position %f, pos=%f\n", joint, -pos - offsetShoulder + startingPositions[4][1], pos);
        // currentPos = odrive5.getPosition(1);
    } else if (joint == 61) {
        odrive6.setPosition(1, -pos - offsetShoulder + startingPositions[5][1]);  // shoulder - left rear
        // SerialMon.printf("Set position for joint %d: axis 1, position %f, pos=%f\n", joint, -pos - offsetShoulder + startingPositions[5][1], pos);
        // currentPos = odrive6.getPosition(1);
    }

    // hips
    else if (joint == 10) {
        odrive1.setPosition(0, pos + offsetHip + startingPositions[0][0]);  // hips - right front
        // SerialMon.printf("Set position for joint %d: axis 0, position %f, pos=%f\n", joint, pos + offsetHip + startingPositions[0][0], pos);
        // currentPos = odrive1.getPosition(0);
    } else if (joint == 11) {
        odrive1.setPosition(1, pos + offsetHip + startingPositions[0][1]);  // hips - right rear
        // SerialMon.printf("Set position for joint %d: axis 1, position %f, pos=%f\n", joint, pos + offsetHip + startingPositions[0][1], pos);
        // currentPos = odrive1.getPosition(1);
    } else if (joint == 40) {
        odrive4.setPosition(0, -pos - offsetHip + startingPositions[3][0]);  // hips - left front
        // SerialMon.printf("Set position for joint %d: axis 0, position %f, pos=%f\n", joint, pos + offsetHip + startingPositions[3][0], pos);
        // currentPos = odrive4.getPosition(0);
    } else if (joint == 41) {
        odrive4.setPosition(1, -pos - offsetHip + startingPositions[3][1]);  // hips - left rear
        // SerialMon.printf("Set position for joint %d: axis 1, position %f, pos=%f\n", joint, pos + offsetHip + startingPositions[3][1], pos);
        // currentPos = odrive4.getPosition(1);
    }

    // SerialMon.printf("Current position for joint %d: position %f\n", joint, currentPos);
}
