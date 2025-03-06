#include <Arduino.h>

// IMU



// #include "common.h"
// #include "kinematics.h"
// #include "ODriveInit.h"
// #include "thresholdSticks.h"


#include "sparky.hpp"
#include "log.hpp"


// bool remoteState = false;




// float RLRFiltered = 0;
// float RFBFiltered = 0;
// float RTFiltered = 340;
// float LLRFiltered = 0;
// float LFBFiltered = 0;
// float LTFiltered = 0;
// int filterFlag1 = 0;

// unsigned long currentMillis;
// long previousMillis = 0;    // set up timers
// long interval = 10;        // time constant for timer
// unsigned long count;

// long previousIMUMillis = 0;    // set up timers

// long previousInterpMillis = 0;    // set up timers
// int interpFlag = 0;

// unsigned long remoteMillis;

// int loopTime;
// int prevLoopTime;

// int stepFlag = 0;
// long previousStepMillis = 0;
// int stepStartFlag = 0;

// int runMode = 0;
// int prevMode = 0;
// bool enabled = false;



// int timer1;   // FB gait timer
// int timer2;   // LR gait timer
// int timer3;   // Yaw gait timer
// float timerScale;   // resulting timer after calcs
// float timerScale2;   // multiplier

// IMU variables

// MPU6050 accelgyro;
// int16_t ax, ay, az;
// int16_t gx, gy, gz;

// #define Gyr_Gain 0.00763358

// float AccelX;
// float AccelY;
// float AccelZ;

// float GyroX;
// float GyroY;
// float GyroZ;

// float mixX;
// float mixY;

// float pitchAccel, rollAccel;

// float IMUpitch;
// float IMUroll;

// Dynamics stability variables

// float legTransX;
// float legTransY;
// float legTransXFiltered;
// float legTransYFiltered;

// float legRoll;
// float legPitch;
// float legRollFiltered;
// float legPitchFiltered;



Sparky sparky;




// ****************** SETUP ******************************
void setup() {
    // logging
    SerialMon.begin(115200);

    if (CrashReport) SerialMon.println(CrashReport);
    Log("init");

    sparky.setup();


    delay(1000);

}

// ********************* MAIN LOOP *******************************
void loop() {
    // return;
    sparky.update();

    return;

    // currentMillis = millis();
    // if (currentMillis - previousMillis >= 10) {  // start timed event

    //     SerialMon.println("==============================");

    //     previousMillis = currentMillis;


        // check loop is actually running the speed we want
        // loopTime = currentMillis - prevLoopTime;
        // prevLoopTime = currentMillis;
        // SerialMon.printf("loop time: %d\n", loopTime);

        // SerialMon.printf("SerialUSB.available %d\n", SerialUSB.available());

        // check for radio data
        // if (SerialUSB.available()) {
        //     // SerialUSB.readBytesUntil('\n', buf, sizeof(buf));
        //     // SerialUSB.readBytes(buf, sizeof(buf));

        //     auto message = MotionProtocol::GetMessage(buf);

        //     // SerialMon.printf("%d\n", message->type());

        //     if (message->type() == MotionProtocol::MessageType::MessageType_REMOTE) {
        //         // SerialMon.println("Recieved remote data");

        //         auto remote_data = message->remote();
        //         remoteMillis = currentMillis;
        //         remoteState = true;

                // if (remote_data->toggle_bottom() == 1) {
                //     // read IMU
                //     Wire.begin();
                //     accelgyro.initialize();
                //     accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

                //     AccelX = ax;
                //     AccelY = ay;
                //     AccelZ = az;
                //     GyroX = Gyr_Gain * (gx);
                //     GyroY = Gyr_Gain * (gy) * -1;
                //     GyroZ = Gyr_Gain * (gz);

                //     AccelY = (atan2(AccelY, AccelZ) * 180 / PI);
                //     AccelX = (atan2(AccelX, AccelZ) * 180 / PI);

                //     float dt = 0.01;
                //     float K = 0.9;
                //     float A = K / (K + dt);

                //     mixX = A * (mixX + GyroX * dt) + (1 - A) * AccelY;
                //     mixY = A * (mixY + GyroY * dt) + (1 - A) * AccelX;

                //     IMUpitch = mixX + 2.7;  // trim IMU to zero
                //     IMUroll = mixY - 5;
                // } else {
                //     // ignore IMU data
                //     Wire.end();
                //     IMUpitch = 0;  // IMU data is zeero, do not read IMU
                //     IMUroll = 0;
                // }


                // int mode = remote_data->mode();

                // if(mode != prevMode) {
                //     prevMode = mode;

                //     if (mode == 1) {  // init ODrives with low gains
                //         SerialMon.println("Init Odrives mode 1");
                //         OdriveInit1();
                //     } else if (mode == 2) {  // default to 45' shoulder and knees
                //         SerialMon.println("Knees mode 2");
                //         // applyOffsets1();
                //     } else if (mode == 3) {  // default to hip position
                //         SerialMon.println("Shoulders mode 3");
                //         // applyOffsets2();
                //     } else if (mode == 4) {  // turn up gains
                //         SerialMon.println("Modify Gains mode 4");
                //         modifyGains();
                //     } else if (mode == 5) {  // turn up gains
                //         SerialMon.println("Kinematics mode 5");
                //         interpFlag = 0;
                //         previousInterpMillis = currentMillis;
                //         runMode = 1;
                //     } else if (mode == 6) {  // turn up gains
                //         SerialMon.println("Walking Mode 6");
                //         interpFlag = 0;
                //         previousInterpMillis = currentMillis;
                //         runMode = 2;
                //     } else if (mode == 9) {  // turn up gains
                //         SerialMon.println("Interp Test");
                //         interpFlag = 0;
                //         previousInterpMillis = currentMillis;
                //         runMode = 9;
                //     } else if (mode == 10) {  // turn up gains
                //         SerialMon.println("Going Home Mode 10");
                //         interpFlag = 0;
                //         previousInterpMillis = currentMillis;
                //         runMode = 10;
                //     }
                // }


                // // Handle motor enable is 0
                // if (remote_data->toggle_top() == 0) {
                //     enabled = false;
                // } else {
                //     enabled = true;
                // }

                // threshold remote data
                // LFB = thresholdStick(remote_data->lfb());
                // LLR = thresholdStick(remote_data->llr());
                // LT = remote_data->lt();
                // RFB = thresholdStick(remote_data->rfb());
                // RLR = thresholdStick(remote_data->rlr());
                // RT = remote_data->rt();

                // SerialMon.printf("lt %d\n", remote_data->lt());

        //         SerialUSB.write("OK\n");
        //     }
        // }

        // // is the remote disconnected for too long ?
        // if (remoteState && currentMillis - remoteMillis > 500) {
        //     remoteState = false;
        //     SerialMon.println("Did not recieve remote data within timeout");
        // }

        // SerialMon.printf("remoteState %d\n", remoteState);

        // stop the dog if the remote becomes disconnected
        // if (!remoteState) {

        // }

        // SerialMon.printf("RFB: %d, RLR: %d, RT: %d, LFB: %d, LLR: %d, LT: %d\n", RFB, RLR, RT, LFB, LLR, LT);

        // if (runMode == 10) {  // put the legs back on the stand

        //     int offset1 = 70;

        //     kinematics(1, -offset1, 0, 270, 0, 0, 0, 0, 0);  // front right
        //     kinematics(2, -offset1, 0, 270, 0, 0, 0, 0, 0);  // front left
        //     kinematics(3, offset1, 0, 270, 0, 0, 0, 0, 0);   // back left
        //     kinematics(4, offset1, 0, 270, 0, 0, 0, 0, 0);   // back right
        // }

        // else if (runMode == 1) {
        //     // ** inverse kinematics demo **

        //     // scale sticks to mm
        //     RFB = map(RFB, -128, 128, -100, 100);
        //     RLR = map(RLR, -128, 128, -100, 100);
        //     RT = map(RT, -128, 128, 240, 440);
        //     RT = constrain(RT, 240, 380);
        //     LFB = map(LFB, -128, 128, -15, 15);
        //     LLR = map(LLR, -128, 128, -15, 15);
        //     LT = map(LT, -128, 128, -20, 20);

        //     // filter sticks
        //     RFBFiltered = filter(RFB, RFBFiltered, 40);
        //     RLRFiltered = filter(RLR, RLRFiltered, 40);
        //     RTFiltered = filter(RT, RTFiltered, 40);
        //     LFBFiltered = filter(LFB, LFBFiltered, 40);
        //     LLRFiltered = filter(LLR, LLRFiltered, 40);
        //     LTFiltered = filter(LT, LTFiltered, 40);

        //     kinematics(1, RFBFiltered, RLRFiltered, RTFiltered, LLRFiltered, LFBFiltered, LTFiltered, 0, 0);  // front right
        //     kinematics(2, RFBFiltered, RLRFiltered, RTFiltered, LLRFiltered, LFBFiltered, LTFiltered, 0, 0);  // front left
        //     kinematics(3, RFBFiltered, RLRFiltered, RTFiltered, LLRFiltered, LFBFiltered, LTFiltered, 0, 0);  // back left
        //     kinematics(4, RFBFiltered, RLRFiltered, RTFiltered, LLRFiltered, LFBFiltered, LTFiltered, 0, 0);  // back right

        // }

        // else if (runMode == 2) {

        // }

    // }  // end of timed loop
}  // end  of main loop
