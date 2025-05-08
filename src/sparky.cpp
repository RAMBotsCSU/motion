#include "sparky.hpp"
#include "config.hpp"
#include "odrive.hpp"
#include "log.hpp"
#include "utils.hpp"
#include "MotionProtocol_generated.h"

Sparky::Sparky()
    : odrive{ODrive(Serial1), ODrive(Serial2), ODrive(Serial3), // mapping for odrives
             ODrive(Serial4), ODrive(Serial5), ODrive(Serial6)},
      leg{{0, odrive[0].axis0, odrive[1].axis1, odrive[1].axis0}, // mapping for leg joint axis
          {1, odrive[3].axis0, odrive[4].axis1, odrive[4].axis0},
          {2, odrive[3].axis1, odrive[5].axis1, odrive[5].axis0},
          {3, odrive[0].axis1, odrive[2].axis1, odrive[2].axis0}},
      kinematics() {}

void Sparky::setup() {
    Log("\n\n== setup ==\n\n");

    //// setup serials
    SerialUSB.begin(115200);
    SerialUSB.setTimeout(0);

    // odrives
    Serial1.begin(115200);
    Serial2.begin(115200);
    Serial3.begin(115200);
    Serial4.begin(115200);
    Serial5.begin(115200);
    Serial6.begin(115200);

    // MPU6050
    mpu.begin();
    mpu.setAccelerometerRange(MPU6050_RANGE_16_G);
    mpu.setGyroRange(MPU6050_RANGE_250_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

    // Attempt to connect all odrives
    for (ODrive& od : odrive) {
        od.connect();
    }
}

unsigned long lastErrCk = 0;
int ODck = 0;


void Sparky::update() {
    unsigned long now = millis();

    if(now - lastTick >= TICK_MS) {
        if(DEBUG) Log("========================== %d %ld\n", now - lastTick, now);

        lastTick = now;

        bool enabled = _enabled;

        // if remote has disconnected
        if (_enabled && now - remoteLastSeen > 500) {
            enabled = false;
            Log("Did not recieve remote data within timeout\n");
        }

        for (ODrive& od : odrive) {
            // check that all odrives are connected
            if(!od.isConnected()) {
                _enabled = false;
                break;
            }

            // check for axis errors
            if(od.axis0.getError() > 0 || od.axis1.getError() > 0) {
                _enabled = false;
                break;
            }
        }

        QuadJointAngles angles;

        if(!enabled) {
            angles = kinematics.home();
        } else {
            if(currentMode == MotionMode::WALK) angles = kinematics.walk(RFB, RLR, LT);
            else if(currentMode == MotionMode::PUSH_UP) angles = kinematics.pushUp(CROSS);
            else if(currentMode == MotionMode::DANCE) angles = kinematics.dance(DPAD_U, DPAD_D, DPAD_L, DPAD_R);
        }

        // sensors_event_t a, g, temp;
        // mpu.getEvent(&a, &g, &temp);

        // Log("Acceleration X: %f, Y: %f, Z: %f m/s^2\n", a.acceleration.x, a.acceleration.y, a.acceleration.z);
        // Log("Rotation X: %f, Y: %f, Z: %f rad/s\n", g.gyro.x, g.gyro.y, g.gyro.z);
        // Log("Temperature: %f degC\n", temp.temperature);


        leg[0].move(angles.FR);
        leg[1].move(angles.FL);
        leg[2].move(angles.BL);
        leg[3].move(angles.BR);
    }

    if(now - lastErrCk >= 100) {
        lastErrCk = now;

        // attempt odrive reconnection if required
        for (ODrive& od : odrive) {
            if(!od.isConnected()) {
                od.connect();
            }
        }

        // check for and attempt to reset axis faults
        Axis* axis;

        if (ODck % 2) axis = &odrive[ODck / 2].axis1;
        else          axis = &odrive[ODck / 2].axis0;

        int err = axis->fetchError();

        Log("odr%d - ax%d: %d\n", ODck / 2, ODck % 2, err);

        if(err > 0) axis->reset();

        if(ODck == 11) ODck = 0;
        else ODck++;
    }



    if (SerialUSB.available()) {
        char controlBuffer[128];
        SerialUSB.readBytes(controlBuffer, sizeof(controlBuffer));

        // Verify the buffer contains a valid message
        flatbuffers::Verifier verifier((const uint8_t*)controlBuffer, sizeof(controlBuffer));
        if (!MotionProtocol::VerifyMessageBuffer(verifier)) {
            Log("Invalid message received\n");
            return;
        }

        auto message = MotionProtocol::GetMessage(controlBuffer);
        if (message == nullptr) {
            Log("Null message received\n");
            return;
        }

        if (message->type() == MotionProtocol::MessageType::MessageType_REMOTE) {
            remoteLastSeen = now;

            auto remote_data = message->remote();

            bool __enabled = remote_data->enabled();

            if(_enabled != __enabled) {
                requestedMode = remote_data->mode();
                _enabled = remote_data->enabled();

                if(_enabled && requestedMode != currentMode) {
                    if(requestedMode == MotionMode::WALK) {
                        setSpeed(1.5);
                    } else if (requestedMode == MotionMode::PUSH_UP) {
                        setSpeed(0.05);
                    } else if (requestedMode == MotionMode::DANCE) {
                        setSpeed(0.2);
                    }

                    kinematics.reset();

                    currentMode = requestedMode;
                }
            }

            LFB = thresholdStick(remote_data->lfb());
            LLR = thresholdStick(remote_data->llr());
            LT = remote_data->lt();
            RFB = thresholdStick(remote_data->rfb());
            RLR = thresholdStick(remote_data->rlr());
            RT = remote_data->rt();

            DPAD_U = remote_data->dpad_u();
            DPAD_D = remote_data->dpad_d();
            DPAD_L = remote_data->dpad_l();
            DPAD_R = remote_data->dpad_r();

            TRIANGLE = remote_data->triangle();
            CROSS = remote_data->cross();
            CIRCLE = remote_data->circle();
            SQUARE = remote_data->square();

            // ODrive status
            flatbuffers::FlatBufferBuilder builder(1024);
            auto odStatus = MotionProtocol::CreateODriveStatus(
                builder,
                odrive[0].isConnected(), odrive[1].isConnected(), odrive[2].isConnected(),
                odrive[3].isConnected(), odrive[4].isConnected(), odrive[5].isConnected(),
                odrive[0].axis0.getError(), odrive[0].axis1.getError(),
                odrive[1].axis0.getError(), odrive[1].axis1.getError(),
                odrive[2].axis0.getError(), odrive[2].axis1.getError(),
                odrive[3].axis0.getError(), odrive[3].axis1.getError(),
                odrive[4].axis0.getError(), odrive[4].axis1.getError(),
                odrive[5].axis0.getError(), odrive[5].axis1.getError()
            );
            builder.Finish(odStatus);

            uint8_t *buf = builder.GetBufferPointer();

            SerialUSB.write(buf, builder.GetSize());
            SerialUSB.write('\n');
        }
    }
}

void Sparky::setSpeed(float speed) {
    for (ODrive& od : odrive) {
        od.axis0.setSpeed(speed);
        od.axis1.setSpeed(speed);
    }
}
