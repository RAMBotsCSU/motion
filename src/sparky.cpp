#include "sparky.hpp"
#include "config.hpp"
#include "odrive.hpp"
#include "log.hpp"
#include "utils.hpp"
#include "MotionProtocol_generated.h"
#include "MPU6050.h"

#define Gyr_Gain 0.00763358
uint8_t rx1Buffer[512];
uint8_t rx2Buffer[512];
uint8_t rx3Buffer[512];
uint8_t rx4Buffer[512];
uint8_t rx5Buffer[512];
uint8_t rx6Buffer[512];

static float IMUpitch = 0.0f;
static float IMUroll  = 0.0f;

static float mixPitch = 0.0f;
static float mixRoll  = 0.0f;

// These values are copied from Bruton's code. They can be adjusted to change the calibrated offsets for the IMU
static const float pitchTrim = 2.7f;
static const float rollTrim  = -5.0f;

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

    Serial1.addMemoryForRead(rx1Buffer, sizeof(rx1Buffer));
    Serial2.addMemoryForRead(rx2Buffer, sizeof(rx2Buffer));
    Serial3.addMemoryForRead(rx3Buffer, sizeof(rx3Buffer));
    Serial4.addMemoryForRead(rx4Buffer, sizeof(rx4Buffer));
    Serial5.addMemoryForRead(rx5Buffer, sizeof(rx5Buffer));
    Serial6.addMemoryForRead(rx6Buffer, sizeof(rx6Buffer));
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

    // Attempt to connect all odrives with a few retries.
    // Single attempts can fail due to serial noise; update() handles any
    // remaining reconnection after setup completes.
    for (int attempt = 0; attempt < 5; attempt++) {
        for (ODrive& od : odrive) {
            if (!od.isConnected()) od.connect();
        }
        bool allConnected = true;
        for (ODrive& od : odrive) {
            if (!od.isConnected()) { allConnected = false; break; }
        }
        if (allConnected) break;
        delay(500);
    }
}

unsigned long lastErrCk = 0;
int ODck = 0;


void Sparky::update() {
    unsigned long now = millis();

    if(now - lastTick >= TICK_MS) {
        if(DEBUG) Log("=== %d %ld\n", now - lastTick, now);

        lastTick = now;

        bool enabled = _enabled;

        // if remote has disconnected
        if (_enabled && now - remoteLastSeen > 500) {
            enabled = false;
            Log("Did not recieve remote data within timeout\n");
        }

        for (int i = 0; i < 6; i++) {
            if(!odrive[i].isConnected()) {
                if(_enabled) Log("[DISABLE] ODrive %d disconnected\n", i);
                _enabled = false;
            }

            if(odrive[i].axis0.getError() > 0 || odrive[i].axis1.getError() > 0) {
                if(_enabled) Log("[DISABLE] ODrive %d axis error: ax0=%lu ax1=%lu\n", i,
                    odrive[i].axis0.getError(), odrive[i].axis1.getError());
                _enabled = false;
            }
        }

        enabled = _enabled;

        QuadJointAngles angles;

        if(!enabled) {
            angles = kinematics.home();
        } else {
            if(currentMode == MotionMode::WALK) angles = kinematics.walk(RFB, RLR, LT, IMUpitch, IMUroll);
            else if(currentMode == MotionMode::PUSH_UP) angles = kinematics.pushUp(CROSS, TRIANGLE, CIRCLE, IMUpitch, IMUroll);
            else if(currentMode == MotionMode::DANCE) angles = kinematics.dance(DPAD_U, DPAD_D, DPAD_L, DPAD_R, IMUpitch, IMUroll);
            //new
            else if(currentMode == MotionMode::LEG_TEST) angles = kinematics.legTesting(TRIANGLE, SQUARE, CROSS, CIRCLE, IMUpitch, IMUroll);
            else if(currentMode == MotionMode::LEG_CONTROL) angles = kinematics.legControl(LLR, LFB, LT, RFB, RLR, RT);
            else angles = kinematics.home();
        }
        
        // sensors_event_t a, g, temp;
        // mpu.getEvent(&a, &g, &temp);

        // Log("Acceleration X: %f, Y: %f, Z: %f m/s^2\n", a.acceleration.x, a.acceleration.y, a.acceleration.z);
        // Log("Rotation X: %f, Y: %f, Z: %f rad/s\n", g.gyro.x, g.gyro.y, g.gyro.z);
        // Log("Temperature: %f degC\n", temp.temperature);


        // ====== IMU update ======
        {
            sensors_event_t a, g, temp;
            mpu.getEvent(&a, &g, &temp);

            // Calculate accel tilt (deg)
            float accelPitch = atan2(a.acceleration.y, a.acceleration.z) * (180.0f / PI);
            float accelRoll  = atan2(a.acceleration.x, a.acceleration.z) * (180.0f / PI);

            // Gyro integration (deg/s * dt)
            float dt = TICK_MS / 1000.0f;
            float gyroPitchRate = g.gyro.x * (180.0f / PI);
            float gyroRollRate  = g.gyro.y * (180.0f / PI);

            // Complementary filter
            const float K = 0.9f;
            const float A = K / (K + dt);

            mixPitch = A * (mixPitch + gyroPitchRate * dt) + (1 - A) * accelPitch;
            mixRoll  = A * (mixRoll  + gyroRollRate  * dt) + (1 - A) * accelRoll;

            // Trimmed values
            IMUpitch = mixPitch + pitchTrim;
            IMUroll  = mixRoll  + rollTrim;

            // (Optional debug)
            // Log("IMU pitch: %f, roll: %f\n", IMUpitch, IMUroll);
        }


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

        if(err > 0) {
            axis->reset();
            // Re-fetch immediately so the cached _error is cleared. Without
            // this, getError() stays non-zero for up to 1.2s (12 axes × 100ms)
            // and keeps _enabled = false in a rapid cycle even though the
            // ODrive has already recovered.
            axis->fetchError();
        }

        // make sure idle axes are forced back into closed-loop
        axis->ensureClosedLoop();

        if(ODck == 11) ODck = 0;
        else ODck++;
    }



    if (SerialUSB.available()) {
        char controlBuffer[128];
        size_t bytesRead = SerialUSB.readBytes(controlBuffer, sizeof(controlBuffer));

        // Verify the buffer contains a valid message using actual bytes read
        flatbuffers::Verifier verifier((const uint8_t*)controlBuffer, bytesRead);
        if (!MotionProtocol::VerifyMessageBuffer(verifier)) {
            Log("Invalid message received\n");
            SerialUSB.flush();
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
                    // Move to HOME before switching modes
                    QuadJointAngles homeAngles = kinematics.home();
                    leg[0].move(homeAngles.FR);
                    leg[1].move(homeAngles.FL);
                    leg[2].move(homeAngles.BL);
                    leg[3].move(homeAngles.BR);

                    // Optionally, add a short delay here if needed for the robot to reach HOME

                    if(requestedMode == MotionMode::WALK) {
                        setSpeed(0.7);
                    } else if (requestedMode == MotionMode::PUSH_UP) {
                        setSpeed(0.06); // Previously 0.05
                    } else if (requestedMode == MotionMode::DANCE) {
                        setSpeed(0.17); // Previously 0.19
                    } else if (requestedMode == MotionMode:: LEG_TEST) {
                        setSpeed(0.06);
                    } else if (requestedMode == MotionMode::LEG_CONTROL){
                        setSpeed(0.7);
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

            // Trigger modifyGains on SQUARE press (rising edge)
            if (SQUARE && !lastSQUARE) {
                modifyGains();
                Log("Modifying ODrive gains\n");
            }
            lastSQUARE = SQUARE;

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
            uint32_t size = builder.GetSize();

            SerialUSB.write((uint8_t*)&size, 4);
            SerialUSB.write(buf, size);
        }
    }
}

void Sparky::setSpeed(float speed) {
    for (ODrive& od : odrive) {
        od.axis0.setSpeed(speed);
        od.axis1.setSpeed(speed);
    }
}

void Sparky::modifyGains() {
    const float posGainKnee     = 17.5;
    const float posGainHips     = 50.0;
    const float posGainShoulder = 20.0;
    const float velGain         = 0.1;
    const float integrator      = 0.08;

    // pos_gain per odrive per axis [odrive][axis]
    const float posGains[6][2] = {
        { posGainHips,  posGainHips     },  // ODrive 0 (Serial1)
        { posGainKnee,  posGainShoulder },  // ODrive 1 (Serial2)
        { posGainKnee,  posGainShoulder },  // ODrive 2 (Serial3)
        { posGainHips,  posGainHips     },  // ODrive 3 (Serial4)
        { posGainKnee,  posGainShoulder },  // ODrive 4 (Serial5)
        { posGainKnee,  posGainShoulder },  // ODrive 5 (Serial6)
    };

    for (int i = 0; i < 6; i++) {
        for (int axis = 0; axis < 2; axis++) {
            odrive[i].send("w axis%d.controller.config.pos_gain %f", axis, posGains[i][axis]);
            odrive[i].send("w axis%d.controller.config.vel_gain %f", axis, velGain);
            odrive[i].send("w axis%d.controller.config.vel_integrator_gain %f", axis, integrator);
        }
    }
}
