#include <Arduino.h>

// IMU
// #include <MPU6050.h>


#include "common.h"
#include "kinematics.h"
#include "ODriveInit.h"
#include "thresholdSticks.h"
#include "MotionProtocol_generated.h"


bool remoteState = false;

int maxLegHeight = 380,
    minLegHeight = 320;

int RLR = 0;
int RFB = 0;
int RT = 0;
int LLR = 0;
int LFB = 0;
int LT = 0;

float RLRFiltered = 0;
float RFBFiltered = 0;
float RTFiltered = 340;
float LLRFiltered = 0;
float LFBFiltered = 0;
float LTFiltered = 0;
int filterFlag1 = 0;

unsigned long currentMillis;
long previousMillis = 0;    // set up timers
long interval = 10;        // time constant for timer
unsigned long count;

long previousIMUMillis = 0;    // set up timers

long previousInterpMillis = 0;    // set up timers
int interpFlag = 0;

unsigned long remoteMillis;

int loopTime;
int prevLoopTime;

int stepFlag = 0;
long previousStepMillis = 0;
int stepStartFlag = 0;

int runMode = 0;
bool enabled = false;

float longLeg1;
float shortLeg1;
float legLength1;
float longLeg2;
float shortLeg2;
float legLength2;
float footOffset;

float fr_RFB;
float fl_RFB;
float bl_RFB;
float br_RFB;
float fr_RLR;
float fl_RLR;
float bl_RLR;
float br_RLR;;
float fr_LT;
float fl_LT;
float bl_LT;
float br_LT;

float fr_LLR;
float fl_LLR;
float br_LLR;
float bl_LLR;

int timer1;   // FB gait timer
int timer2;   // LR gait timer
int timer3;   // Yaw gait timer
float timerScale;   // resulting timer after calcs
float timerScale2;   // multiplier


// IMU variables

// MPU6050 accelgyro;
// int16_t ax, ay, az;
// int16_t gx, gy, gz;

// #define Gyr_Gain 0.00763358

float AccelX;
float AccelY;
float AccelZ;

float GyroX;
float GyroY;
float GyroZ;

float mixX;
float mixY;

float pitchAccel, rollAccel;

float IMUpitch;
float IMUroll;

// Dynamics stability variables

float legTransX;
float legTransY;
float legTransXFiltered;
float legTransYFiltered;

float legRoll;
float legPitch;
float legRollFiltered;
float legPitchFiltered;


// ****************** SETUP ******************************
void setup() {
    // initialize serial communication
    SerialUSB.begin(115200);
    SerialUSB.setTimeout(0);

    Serial1.begin(115200);
    Serial2.begin(115200);
    Serial3.begin(115200);
    Serial4.begin(115200);
    Serial5.begin(115200);
    Serial6.begin(115200);

    SerialMon.begin(115200);

    // LCD
    // lcd.init();
    // lcd.backlight();
    // lcd.setCursor(0, 0);
    // lcd.print("openDog V3");
    // lcd.setCursor(0, 1);
    // lcd.print("S:0  C:0");

    SerialMon.println("init");

    if (CrashReport) SerialMon.print(CrashReport);

    delay(1000);

    OdriveInit1();

    delay(1000);

    // applyOffsets1();
    // applyOffsets2();

    // delay(1000);

}  // end of setup

// ********************* MAIN LOOP *******************************

void loop() {
    currentMillis = millis();
    if (currentMillis - previousMillis >= 10) {  // start timed event

        SerialMon.println("==============================");

        previousMillis = currentMillis;

        // check loop is actually running the speed we want
        // loopTime = currentMillis - prevLoopTime;
        // prevLoopTime = currentMillis;
        // SerialMon.printf("loop time: %d\n", loopTime);

        SerialMon.printf("SerialUSB.available %d\n", SerialUSB.available());

        // check for radio data
        if (SerialUSB.available()) {
            char buf[128];
            // SerialUSB.readBytesUntil('\n', buf, sizeof(buf));
            SerialUSB.readBytes(buf, sizeof(buf));

            auto message = MotionProtocol::GetMessage(buf);

            // SerialMon.printf("%d\n", message->type());

            if (message->type() == MotionProtocol::MessageType::MessageType_REMOTE) {
                SerialMon.println("Recieved remote data");

                auto remote_data = message->remote();
                remoteMillis = currentMillis;
                remoteState = true;

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


                int mode = remote_data->mode();

                if (mode == 1) {  // init ODrives with low gains
                    SerialMon.println("Init Odrives mode 1");
                    OdriveInit1();
                } else if (mode == 2) {  // default to 45' shoulder and knees
                    SerialMon.println("Knees mode 2");
                    applyOffsets1();
                } else if (mode == 3) {  // default to hip position
                    SerialMon.println("Shoulders mode 3");
                    applyOffsets2();
                } else if (mode == 4) {  // turn up gains
                    SerialMon.println("Modify Gains mode 4");
                    modifyGains();
                } else if (mode == 5) {  // turn up gains
                    SerialMon.println("Kinematics mode 5");
                    interpFlag = 0;
                    previousInterpMillis = currentMillis;
                    runMode = 1;
                } else if (mode == 6) {  // turn up gains
                    SerialMon.println("Walking Mode 6");
                    interpFlag = 0;
                    previousInterpMillis = currentMillis;
                    runMode = 2;
                } else if (mode == 9) {  // turn up gains
                    SerialMon.println("Interp Test");
                    interpFlag = 0;
                    previousInterpMillis = currentMillis;
                    runMode = 9;
                } else if (mode == 10) {  // turn up gains
                    SerialMon.println("Going Home Mode 10");
                    interpFlag = 0;
                    previousInterpMillis = currentMillis;
                    runMode = 10;
                }


                // Handle motor enable is 0
                if (remote_data->toggle_top() == 0) {
                    enabled = false;
                } else {
                    enabled = true;
                }

                // threshold remote data
                LFB = thresholdStick(remote_data->lfb());
                LLR = thresholdStick(remote_data->llr());
                LT = remote_data->lt();
                RFB = thresholdStick(remote_data->rfb());
                RLR = thresholdStick(remote_data->rlr());
                RT = remote_data->rt();

                SerialMon.printf("lt %d\n", remote_data->lt());

                SerialUSB.write("OK\n");
            }
        }

        // // is the remote disconnected for too long ?
        if (remoteState && currentMillis - remoteMillis > 500) {
            remoteState = false;
            SerialMon.println("Did not recieve remote data within timeout");
        }

        SerialMon.printf("remoteState %d\n", remoteState);

        // stop the dog if the remote becomes disconnected
        if (!remoteState) {
            RFB = 0;
            RLR = 0;
            RT = 0;
            LFB = 0;
            LLR = 0;
            LT = 0;
        }

        SerialMon.printf("RFB: %d, RLR: %d, RT: %d, LFB: %d, LLR: %d, LT: %d\n", RFB, RLR, RT, LFB, LLR, LT);

        if (runMode == 10) {  // put the legs back on the stand

            int offset1 = 70;

            kinematics(1, -offset1, 0, 270, 0, 0, 0, 0, 0);  // front right
            kinematics(2, -offset1, 0, 270, 0, 0, 0, 0, 0);  // front left
            kinematics(3, offset1, 0, 270, 0, 0, 0, 0, 0);   // back left
            kinematics(4, offset1, 0, 270, 0, 0, 0, 0, 0);   // back right
        }

        else if (runMode == 1) {
            // ** inverse kinematics demo **

            // scale sticks to mm
            RFB = map(RFB, -128, 128, -100, 100);
            RLR = map(RLR, -128, 128, -100, 100);
            RT = map(RT, -128, 128, 240, 440);
            RT = constrain(RT, 240, 380);
            LFB = map(LFB, -128, 128, -15, 15);
            LLR = map(LLR, -128, 128, -15, 15);
            LT = map(LT, -128, 128, -20, 20);

            // filter sticks
            RFBFiltered = filter(RFB, RFBFiltered, 40);
            RLRFiltered = filter(RLR, RLRFiltered, 40);
            RTFiltered = filter(RT, RTFiltered, 40);
            LFBFiltered = filter(LFB, LFBFiltered, 40);
            LLRFiltered = filter(LLR, LLRFiltered, 40);
            LTFiltered = filter(LT, LTFiltered, 40);

            kinematics(1, RFBFiltered, RLRFiltered, RTFiltered, LLRFiltered, LFBFiltered, LTFiltered, 0, 0);  // front right
            kinematics(2, RFBFiltered, RLRFiltered, RTFiltered, LLRFiltered, LFBFiltered, LTFiltered, 0, 0);  // front left
            kinematics(3, RFBFiltered, RLRFiltered, RTFiltered, LLRFiltered, LFBFiltered, LTFiltered, 0, 0);  // back left
            kinematics(4, RFBFiltered, RLRFiltered, RTFiltered, LLRFiltered, LFBFiltered, LTFiltered, 0, 0);  // back right

        }

        else if (runMode == 2) {
            // simple walking

            RFB = map(RFB, -128, 128, -50, 50);  // mm
            RLR = map(RLR, -128, 128, -25, 25);  // mm
            LT = map(LT, 0, 255, 0, 25);    // degrees

            RFBFiltered = filter(RFB, RFBFiltered, 15);
            RLRFiltered = filter(RLR, RLRFiltered, 15);
            LTFiltered = filter(LT, LTFiltered, 15);

            longLeg1 = maxLegHeight;
            shortLeg1 = minLegHeight;
            longLeg2 = maxLegHeight;
            shortLeg2 = minLegHeight;

            footOffset = 0;
            timer1 = 80;  // FB gait timer  80
            // timer2 = 75;   // LR gait timer
            // timer3 = 75;   // LR gait timer

            SerialMon.printf("RFBFiltered: %f RLRFiltered: %f LTFiltered: %f\n", RFBFiltered, RLRFiltered, LTFiltered);

            if (abs(RFBFiltered) < 0.1 && abs(RLRFiltered) < 0.1 && abs(LTFiltered) < 0.1) {  // controls are centred or near enough
                SerialMon.println("STANDING STILL");

                // position legs a default standing positionS
                legLength1 = longLeg1;
                legLength2 = longLeg2;
                fr_RFB = 0;
                fl_RFB = 0;
                bl_RFB = 0;
                br_RFB = 0;
                fr_RLR = footOffset;
                fl_RLR = -footOffset;
                bl_RLR = -footOffset;
                br_RLR = footOffset;
                fr_LT = 0;
                fl_LT = 0;
                bl_LT = 0;
                br_LT = 0;
            }

            // walking
            else {
                SerialMon.printf("WALKING - %d\n", stepFlag);

                if (stepFlag == 0 && currentMillis - previousStepMillis > timerScale) {
                    legLength1 = shortLeg1;
                    legLength2 = longLeg2;
                    fr_RFB = 0 - RFBFiltered;
                    fl_RFB = RFBFiltered;
                    bl_RFB = 0 - RFBFiltered;
                    br_RFB = RFBFiltered;
                    fr_RLR = (footOffset - RLRFiltered) + LT;
                    fl_RLR = (-footOffset + RLRFiltered) - LT;
                    bl_RLR = (-footOffset - RLRFiltered) - LT;
                    br_RLR = (footOffset + RLRFiltered) + LT;
                    // fr_RLR = LT;
                    // fl_RLR = 0-LT;
                    // bl_RLR = 0-LT;
                    // br_RLR = LT;
                    stepFlag = 1;
                    previousStepMillis = currentMillis;
                }

                else if (stepFlag == 1 && currentMillis - previousStepMillis > timerScale) {
                    legLength1 = longLeg1;
                    legLength2 = longLeg2;
                    fr_RFB = 0 - RFBFiltered;
                    fl_RFB = RFBFiltered;
                    bl_RFB = 0 - RFBFiltered;
                    br_RFB = RFBFiltered;
                    fr_RLR = (footOffset - RLRFiltered) + LT;
                    fl_RLR = (-footOffset + RLRFiltered) - LT;
                    bl_RLR = (-footOffset - RLRFiltered) - LT;
                    br_RLR = (footOffset + RLRFiltered) + LT;
                    // fr_RLR = LT;
                    // fl_RLR = 0-LT;
                    // bl_RLR = 0-LT;
                    // br_RLR = LT;

                    stepFlag = 2;
                    previousStepMillis = currentMillis;
                }

                else if (stepFlag == 2 && currentMillis - previousStepMillis > timerScale) {
                    legLength1 = longLeg1;
                    legLength2 = shortLeg2;
                    fr_RFB = RFBFiltered;
                    fl_RFB = 0 - RFBFiltered;
                    bl_RFB = RFBFiltered;
                    br_RFB = 0 - RFBFiltered;
                    fr_RLR = (footOffset + RLRFiltered) - LT;
                    fl_RLR = (-footOffset - RLRFiltered) + LT;
                    bl_RLR = (-footOffset + RLRFiltered) + LT;
                    br_RLR = (footOffset - RLRFiltered) - LT;
                    // fr_RLR = 0-LT;
                    // fl_RLR = LT;
                    // bl_RLR = LT;
                    //  br_RLR = 0-LT;
                    stepFlag = 3;
                    previousStepMillis = currentMillis;
                }

                else if (stepFlag == 3 && currentMillis - previousStepMillis > timerScale) {
                    legLength1 = longLeg1;
                    legLength2 = longLeg2;
                    fr_RFB = RFBFiltered;
                    fl_RFB = 0 - RFBFiltered;
                    bl_RFB = RFBFiltered;
                    br_RFB = 0 - RFBFiltered;
                    fr_RLR = (footOffset + RLRFiltered) - LT;
                    fl_RLR = (-footOffset - RLRFiltered) + LT;
                    bl_RLR = (-footOffset + RLRFiltered) + LT;
                    br_RLR = (footOffset - RLRFiltered) - LT;
                    // fr_RLR = 0-LT;
                    // fl_RLR = LT;
                    // bl_RLR = LT;
                    // br_RLR = 0-LT;
                    stepFlag = 0;
                    previousStepMillis = currentMillis;
                }

                float stepLength;
                float stepWidth;
                float stepAngle;
                float stepHyp;

                // timer calcs

                stepLength = abs(fr_RFB);
                stepWidth = abs(fr_RLR);

                if (stepLength == 0.0) {
                    stepLength = 0.01;  // avoid divide by zero
                }

                stepAngle = atan(stepLength / stepWidth);    // radians       // work out actual distance of step
                stepHyp = abs(stepLength / sin(stepAngle));  // mm

                timerScale = timer1 + (stepHyp / 3.5);
            }

            legTransX = IMUpitch * -2;
            legTransY = IMUroll * -2;

            legTransXFiltered = filter(legTransX, legTransXFiltered, 50);
            legTransYFiltered = filter(legTransY, legTransYFiltered, 50);

            legRoll = IMUroll * -0.5;
            legPitch = IMUpitch * 0.5;

            legRollFiltered = filter(legRoll, legRollFiltered, 60);
            legPitchFiltered = filter(legPitch, legPitchFiltered, 60);

            kinematics(1, fr_RFB - legTransXFiltered, fr_RLR - legTransYFiltered, legLength1, legRollFiltered, legPitchFiltered, 0, 1, (timerScale * 0.8));  // front right
            kinematics(2, fl_RFB - legTransXFiltered, fl_RLR - legTransYFiltered, legLength2, legRollFiltered, legPitchFiltered, 0, 1, (timerScale * 0.8));  // front left
            kinematics(3, bl_RFB - legTransXFiltered, bl_RLR - legTransYFiltered, legLength1, legRollFiltered, legPitchFiltered, 0, 1, (timerScale * 0.8));  // back left
            kinematics(4, br_RFB - legTransXFiltered, br_RLR - legTransYFiltered, legLength2, legRollFiltered, legPitchFiltered, 0, 1, (timerScale * 0.8));  // back right
        }
    }  // end of timed loop
}  // end  of main loop
