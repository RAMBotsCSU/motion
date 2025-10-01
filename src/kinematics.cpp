#include <Arduino.h>

#include "kinematics.hpp"
#include "config.hpp"
#include "log.hpp"
#include "utils.hpp"

// factor for converting degrees to motor turns used by the ODrive
#define CONVERSION 0.02777777777777777777777777777778

// zero position offsets for when the robot is standing still
#define OFFSET_SHOULDER 0.744587
#define OFFSET_KNEE -1.489173
#define OFFSET_HIP 0.000006


JointAngles Kinematics::translate(int leg, float xIn, float yIn, float zIn, float roll, float pitch, float yawIn) {
    // Log("[KINEMATICS] leg: %d, xIn: %.2f, yIn: %.2f, zIn: %.2f, roll: %.2f, pitch: %.2f, yawIn: %.2f, interOn: %d, dur: %d\n", leg, xIn, yIn, zIn, roll, pitch, yawIn, interOn, dur);

    // leg 1  : front right
    // leg 2  : front left
    // leg 3  : back left
    // leg 4  : back right

    // moving the foot sideways on the end plane
    float hipOffset = 108;  // distance from the hip pivot to the centre of the leg
    float hipAngle1a;
    float hipAngle1b;
    float hipAngle1;
    float hipAngle1Degrees;
    float hipHyp;

    // moving the foot forwards or backwardes in the side plane
    float shoulderAngle2;
    float shoulderAngle2Degrees;
    float z2;

    // side plane of individual leg only
    const int shinLength = 200;
    const int thighLength = 200;
    float z3;
    float shoulderAngle1;
    float shoulderAngle1Degrees;
    float shoulderAngle1a;
    float shoulderAngle1b;
    float shoulderAngle1c;
    float kneeAngle;
    float kneeAngleDegrees;

    // *** ROTATION AXIS

    // roll axis
    const int bodyWidth = 59;         // half the distance between the hip  pivots (the front)
    float legDiffRoll;                // differnece in height for each leg
    float bodyDiffRoll;               // how much shorter the 'virtual body' gets
    float footDisplacementRoll;       // where the foot actually is
    float footDisplacementAngleRoll;  // smaller angle
    float footWholeAngleRoll;         // whole leg angle
    float rollAngle;                  // angle in RADIANS that the body rolls
    float zz1a;                       // hypotenuse of final triangle
    float zz1;                        // new height for leg to pass onto the next bit of code
    float yy1;                        // new position for leg to move sideways

    // pitch axis
    const int bodyLength = 272;        // half the distance between shoulder pivots  (the side)
    float legDiffPitch;                // differnece in height for each leg
    float bodyDiffPitch;               // how much shorter the 'virtual body' gets
    float footDisplacementPitch;       // where the foot actually is
    float footDisplacementAnglePitch;  // smaller angle
    float footWholeAnglePitch;         // whole leg angle
    float pitchAngle;                  // angle in RADIANS that the body rolls
    float zz2a;                        // hypotenuse of final triangle
    float zz2;                         // new height for the leg to pass onto the next bit of code
    float xx1;                         // new position to move the leg fowwards/backwards

    // yaw axis

    float yawAngle;       // angle in RADIANs for rotation in yaw
    float existingAngle;  // existing angle of leg from centre
    float radius;         // radius of leg from centre of robot based on x and y from sticks
    float demandYaw;      // demand yaw postion - existing yaw plus the stick yaw
    float xx3;            // new X coordinate based on demand angle
    float yy3;            // new Y coordinate based on demand angle

    float z = zIn;
    float x = xIn;
    float y = yIn;
    float yaw = yawIn;

    // **** START INVERSE KINEMATICS CALCS ****

    // ** YAW AXIS **

    // convert degrees to radians for the calcs
    yawAngle = (PI / 180) * yaw;

    // put in offsets from robot's parameters so we can work out the radius of the foot from the robot's centre
    if (leg == 1) {  // front left leg
        y = y - (bodyWidth + hipOffset);
        x = x - bodyLength;
    } else if (leg == 2) {  // front right leg
        y = y + (bodyWidth + hipOffset);
        x = x - bodyLength;
    } else if (leg == 3) {  // back left leg
        y = y - (bodyWidth + hipOffset);
        x = x + bodyLength;
    } else if (leg == 4) {  // back left leg
        y = y + (bodyWidth + hipOffset);
        x = x + bodyLength;
    }

    // calc existing angle of leg from center
    existingAngle = atan(y / x);

    // calc radius from centre
    radius = y / sin(existingAngle);

    // calc demand yaw angle
    demandYaw = existingAngle + yawAngle;

    // calc new X and Y based on demand yaw angle
    xx3 = radius * cos(demandYaw);
    yy3 = radius * sin(demandYaw);

    // remove the offsets so we pivot around 0/0 x/y
    if (leg == 1) {  // front left leg
        yy3 = yy3 + (bodyWidth + hipOffset);
        xx3 = xx3 + bodyLength;
    } else if (leg == 2) {  // front right leg
        yy3 = yy3 - (bodyWidth + hipOffset);
        xx3 = xx3 + bodyLength;
    } else if (leg == 3) {  // back left leg
        yy3 = yy3 + (bodyWidth + hipOffset);
        xx3 = xx3 - bodyLength;
    } else if (leg == 4) {  // back left leg
        yy3 = yy3 - (bodyWidth + hipOffset);
        xx3 = xx3 - bodyLength;
    }

    // ** PITCH AXIS ***

    if (leg == 1 || leg == 2) {
        pitch = -pitch;
        xx3 = -xx3;
    }

    // convert pitch to degrees
    pitchAngle = (PI / 180) * pitch;

    // calc top triangle sides
    legDiffPitch = sin(pitchAngle) * bodyLength;
    bodyDiffPitch = cos(pitchAngle) * bodyLength;

    // calc actual height from the ground for each side
    legDiffPitch = z - legDiffPitch;

    // calc foot displacement
    footDisplacementPitch = ((bodyDiffPitch - bodyLength) * -1) + xx3;

    // calc smaller displacement angle
    footDisplacementAnglePitch = atan(footDisplacementPitch / legDiffPitch);

    // calc distance from the ground at the displacement angle (the hypotenuse of the final triangle)
    zz2a = legDiffPitch / cos(footDisplacementAnglePitch);

    // calc the whole angle for the leg
    footWholeAnglePitch = footDisplacementAnglePitch + pitchAngle;

    // calc actual leg length - the new Z to pass on
    zz2 = cos(footWholeAnglePitch) * zz2a;

    // calc new Z to pass on
    xx1 = sin(footWholeAnglePitch) * zz2a;

    if (leg == 1 || leg == 4) {
        xx1 = -xx1;
    }

    // *** ROLL AXIS ***

    // turn around roll angle for each side of the robot
    if (leg == 2 || leg == 3) {
        yy3 = -yy3;
    }

    // convert roll angle to radians
    rollAngle = (PI / 180) * roll;

    // calc the top triangle sides
    legDiffRoll = sin(rollAngle) * bodyWidth;
    bodyDiffRoll = cos(rollAngle) * bodyWidth;

    // calc actual height from the ground for each side
    legDiffRoll = zz2 - legDiffRoll;

    // calc foot displacement
    footDisplacementRoll = (((bodyDiffRoll - bodyWidth) * -1) + hipOffset) - yy3;

    // calc smaller displacement angle
    footDisplacementAngleRoll = atan(footDisplacementRoll / legDiffRoll);

    // calc distance from the ground at the displacement angle (the hypotenuse of the final triangle)
    zz1a = legDiffRoll / cos(footDisplacementAngleRoll);

    // calc the whole angle for the leg
    footWholeAngleRoll = footDisplacementAngleRoll + rollAngle;

    // calc actual leg length - the new Z to pass on
    zz1 = cos(footWholeAngleRoll) * zz1a;

    // calc new Y to pass on
    yy1 = (sin(footWholeAngleRoll) * zz1a) - hipOffset;  // take away the offset so we can pivot around zero

    // *** TRANSLATION AXIS ***

    // calculate the hip joint and new leg length based on how far the robot moves sideways
    // Y axis - side to side
    // first triangle

    if (leg == 1 || leg == 4) {  // reverse the calcs for each side of the robot
        hipOffset = -hipOffset;
        yy1 = -yy1;
    }

    yy1 = yy1 + hipOffset;  // add on hip offset because there is default distance in Y
    hipAngle1a = atan(yy1 / zz1);
    hipAngle1Degrees = ((hipAngle1a * (180 / PI)));  // convert to degrees
    hipHyp = zz1 / cos(hipAngle1a);                  // this is the hypotenuse of the first triangle

    // second triangle

    hipAngle1b = asin(hipOffset / hipHyp);                  // calc 'the other angle' in the triangle
    hipAngle1 = (PI - (PI / 2) - hipAngle1b) + hipAngle1a;  // calc total hip angle
    hipAngle1 = hipAngle1 - 1.5708;                         // take away offset for rest position
    hipAngle1Degrees = ((hipAngle1 * (180 / PI)));          // convert to degrees

    // calc new leg length to give to the code  below
    z2 = hipOffset / tan(hipAngle1b);  // new leg length

    // ****************

    // X axis - front to back
    // calculate the shoulder joint offset and new leg length based on now far the foot moves forward/backwards
    shoulderAngle2 = atan(xx1 / z2);  // calc how much extra to add to the shoulder joint
    shoulderAngle2Degrees = shoulderAngle2 * (180 / PI);
    z3 = z2 / cos(shoulderAngle2);  // calc new leg length to feed to the next bit of code below

    // ****************

    // Z axis - up and down
    // calculate leg length based on shin/thigh length and knee and shoulder angle
    z3 = constrain(z3, 200, 390);  // constrain leg length to stop it turning inside out and breaking the trig
    shoulderAngle1a = sq(thighLength) + sq(z3) - sq(shinLength);
    shoulderAngle1b = 2 * thighLength * z3;
    shoulderAngle1c = shoulderAngle1a / shoulderAngle1b;
    shoulderAngle1 = acos(shoulderAngle1c);  // radians
    kneeAngle = PI - (shoulderAngle1 * 2);   // radians

    // calc degrees from angles
    shoulderAngle1Degrees = shoulderAngle1 * (180 / PI);  // degrees
    kneeAngleDegrees = kneeAngle * (180 / PI);            // degrees

    float shoulderAngle1Counts, shoulderAngle2Counts;

    // Convert angles to motor rotations, and apply zero offsets

    JointAngles angles;

    if (leg == 1) { // front right
        shoulderAngle1Counts = (shoulderAngle1Degrees - 45) * CONVERSION;
        shoulderAngle2Counts = shoulderAngle2Degrees * CONVERSION;
        angles.shoulder = -shoulderAngle1Counts + shoulderAngle2Counts - OFFSET_SHOULDER;
        angles.knee = (kneeAngleDegrees - 90) * CONVERSION + OFFSET_KNEE;
        angles.hip = -hipAngle1Degrees * CONVERSION - OFFSET_HIP;
    }

    else if (leg == 2) { // front left
        shoulderAngle1Counts = (shoulderAngle1Degrees - 45) * CONVERSION;
        shoulderAngle2Counts = shoulderAngle2Degrees * CONVERSION;
        angles.shoulder = shoulderAngle1Counts + shoulderAngle2Counts + OFFSET_SHOULDER;
        angles.knee = -(kneeAngleDegrees - 90) * CONVERSION - OFFSET_KNEE;
        angles.hip = hipAngle1Degrees * CONVERSION + OFFSET_HIP;
    }

    else if (leg == 3) { // back left
        shoulderAngle1Counts = (shoulderAngle1Degrees - 45) * CONVERSION;
        shoulderAngle2Counts = shoulderAngle2Degrees * CONVERSION;
        angles.shoulder = -shoulderAngle1Counts - shoulderAngle2Counts - OFFSET_SHOULDER;
        angles.knee = (kneeAngleDegrees - 90) * CONVERSION + OFFSET_KNEE;
        angles.hip = -hipAngle1Degrees * CONVERSION - OFFSET_HIP;
    }

    else if (leg == 4) { // back right
        shoulderAngle1Counts = (shoulderAngle1Degrees - 45) * CONVERSION;
        shoulderAngle2Counts = shoulderAngle2Degrees * CONVERSION;
        angles.shoulder = shoulderAngle1Counts - shoulderAngle2Counts + OFFSET_SHOULDER;
        angles.knee = -(kneeAngleDegrees - 90) * CONVERSION - OFFSET_KNEE;
        angles.hip = hipAngle1Degrees * CONVERSION + OFFSET_HIP;
    }

    return angles;
}

void Kinematics::reset() {
    step = 0;
    lastStepAt = 0;
}

const int maxLegHeight = 380,
    minLegHeight = 320;

// simple walking
QuadJointAngles Kinematics::walk(int RFB, int RLR, int LT, float IMUpitch, float IMUroll) {
    unsigned long now = millis();

    RFB = map(RFB, -128, 128, -40, 40);  // mm
    RLR = map(RLR, -128, 128, -20, 20);  // mm
    LT = map(LT, 0, 255, 0, 25);    // degrees

    lastRFB = RFB;
    lastRLR = RLR;
    lastLT = LT;

    int longLeg1 = maxLegHeight,
        shortLeg1 = minLegHeight,
        longLeg2 = maxLegHeight,
        shortLeg2 = minLegHeight;

    int footOffset = 0;
    int timer1 = 80 * 1.0 / 0.7 * (1.0f / GLOBAL_SPEED);  // FB gait timer  80

    // Log("lastRFB: %f lastRLR: %f LTFiltered: %f\n", lastRFB, lastRLR, LTFiltered);

    float legLength1 = 0,
        legLength2 = 0;

    float fr_RFB = 0,
        fl_RFB = 0,
        bl_RFB = 0,
        br_RFB = 0,
        fr_RLR = 0,
        fl_RLR = 0,
        bl_RLR = 0,
        br_RLR = 0;
    float timerScale = 0;

    if (lastRFB == 0 && lastRLR == 0 && lastLT == 0) {  // controls are centered
        Log("[WALKSTATUS] STANDING STILL\n");

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
    }

    // walking
    else {
        Log("[WALKSTATUS] WALKING - %d\n", step);

        if (step == 0) {
            legLength1 = shortLeg1;
            legLength2 = longLeg2;
            fr_RFB = -lastRFB;
            fl_RFB = lastRFB;
            bl_RFB = -lastRFB;
            br_RFB = lastRFB;
            fr_RLR = (footOffset - lastRLR) + LT;
            fl_RLR = (-footOffset + lastRLR) - LT;
            bl_RLR = (-footOffset - lastRLR) - LT;
            br_RLR = (footOffset + lastRLR) + LT;
        }

        else if (step == 1) {
            legLength1 = longLeg1;
            legLength2 = longLeg2;
            fr_RFB = -lastRFB;
            fl_RFB = lastRFB;
            bl_RFB = -lastRFB;
            br_RFB = lastRFB;
            fr_RLR = (footOffset - lastRLR) + LT;
            fl_RLR = (-footOffset + lastRLR) - LT;
            bl_RLR = (-footOffset - lastRLR) - LT;
            br_RLR = (footOffset + lastRLR) + LT;
        }

        else if (step == 2) {
            legLength1 = longLeg1;
            legLength2 = shortLeg2;
            fr_RFB = lastRFB;
            fl_RFB = -lastRFB;
            bl_RFB = lastRFB;
            br_RFB = -lastRFB;
            fr_RLR = (footOffset + lastRLR) - LT;
            fl_RLR = (-footOffset - lastRLR) + LT;
            bl_RLR = (-footOffset + lastRLR) + LT;
            br_RLR = (footOffset - lastRLR) - LT;
        }

        else if (step == 3) {
            legLength1 = longLeg1;
            legLength2 = longLeg2;
            fr_RFB = lastRFB;
            fl_RFB = -lastRFB;
            bl_RFB = lastRFB;
            br_RFB = -lastRFB;
            fr_RLR = (footOffset + lastRLR) - LT;
            fl_RLR = (-footOffset - lastRLR) + LT;
            bl_RLR = (-footOffset + lastRLR) + LT;
            br_RLR = (footOffset - lastRLR) - LT;
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

        if(now - lastStepAt > timerScale) {
            lastStepAt = now;
            if(step == 3) step = 0;
            else step++;
        }

        // Log("=== stepHyp: %f, timerScale: %f\n", stepHyp, timerScale);
    }

    float legTransX = IMUpitch * -2.0f;
    float legTransY = IMUroll  * -2.0f;

    float legRoll   = IMUroll  * -0.5f;
    float legPitch  = IMUpitch *  0.5f;

    QuadJointAngles angles = {
        translate(1, fr_RFB - legTransX, fr_RLR - legTransY, legLength1, legRoll, legPitch, 0),
        translate(2, fl_RFB - legTransX, fl_RLR - legTransY, legLength2, legRoll, legPitch, 0),
        translate(3, bl_RFB - legTransX, bl_RLR - legTransY, legLength1, legRoll, legPitch, 0),
        translate(4, br_RFB - legTransX, br_RLR - legTransY, legLength2, legRoll, legPitch, 0),
    };

    return angles;
}

// push ups and sitting
// If triangle is pressed, sparky sits. If cross is pressed, sparky goes down on all 4 legs. If both are pressed, sparky goes down on all 4 legs.
QuadJointAngles Kinematics::pushUp(bool cross_press, bool triangle_press, float IMUpitch, float IMUroll) {
    int pushUpPos = maxLegHeight;
    int sitPos = maxLegHeight;

    float corrRoll = IMUroll * -03f; // correct for roll
    float corrPitch = IMUpitch * -0.3f; // correct for pitch

    QuadJointAngles angles = {};

    if (triangle_press) { // back legs go down
      sitPos = maxLegHeight - 20;

      angles = {
        translate (1, 0, 0, pushUpPos, corrRoll, corrPitch, 0),
        translate (2, 0, 0, pushUpPos, corrRoll, corrPitch, 0),
        translate (3, 0, 0, sitPos, corrRoll, corrPitch, 0),
        translate (4, 0, 0, sitPos, corrRoll, corrPitch, 0),
      };
    }

    if (cross_press) { // all legs go down
      pushUpPos = maxLegHeight - 20;
      
      angles = {
        translate (1, 0, 0, pushUpPos, corrRoll, corrPitch, 0),
        translate (2, 0, 0, pushUpPos, corrRoll, corrPitch, 0),
        translate (3, 0, 0, pushUpPos, corrRoll, corrPitch, 0),
        translate (4, 0, 0, pushUpPos, corrRoll, corrPitch, 0),
      };
    }

    return angles;
}

// dance
bool dancing = false;
float dancePos = 0.0f;
unsigned int danceTimer = 250;
QuadJointAngles Kinematics::dance(bool up, bool down, bool left, bool right) {
    unsigned long now = millis();

    // Log("=== %d %d %d %d\n", up, down, left, right);

    QuadJointAngles angles;

    if (dancing) {
        if(now - lastStepAt > danceTimer) {
            lastStepAt = now;
            step = (step + 1) % 4;
        }
    }

    if (up) { // Rock and roll (sways right to left)
        dancing = true;

        if (step % 2 == 0) {
            dancePos = 7;
        } else {
            dancePos = -7;
        }

        angles = {
            translate(1, 0, 0, maxLegHeight, dancePos + IMUroll * -0.2f, IMUpitch * 0.2f, 0),
            translate(2, 0, 0, maxLegHeight, dancePos + IMUroll * -0.2f, IMUpitch * 0.2f, 0),
            translate(3, 0, 0, maxLegHeight, dancePos + IMUroll * -0.2f, IMUpitch * 0.2f, 0),
            translate(4, 0, 0, maxLegHeight, dancePos + IMUroll * -0.2f, IMUpitch * 0.2f, 0),
        };
    }

    else if (right) { // Head bob (sways forward and backward)
        dancing = true;

        if (step % 2 == 0) {
            dancePos = 5;
        } else {
            dancePos = -5;
        }

        angles = {
            translate(1, 0, 0, maxLegHeight, 0, dancePos, 0),
            translate(2, 0, 0, maxLegHeight, 0, dancePos, 0),
            translate(3, 0, 0, maxLegHeight, 0, dancePos, 0),
            translate(4, 0, 0, maxLegHeight, 0, dancePos, 0),
        };
    }

    else if (left) { // Bop (all legs go up and down)
        dancing = true;

        if (step % 2 == 0) {
            dancePos = maxLegHeight + 5;
        } else {
            dancePos = maxLegHeight - 5;
        }

        angles = {
            translate(1, 0, 0, dancePos, 0, 0, 0),
            translate(2, 0, 0, dancePos, 0, 0, 0),
            translate(3, 0, 0, dancePos, 0, 0, 0),
            translate(4, 0, 0, dancePos, 0, 0, 0),
        };
    }

    else if (down) {
        dancing = true;
        int dancePos2;

        if (step==0) {
            dancePos = maxLegHeight - 10;
            dancePos2 = maxLegHeight + 5;
        } else if (step==1){
            dancePos = maxLegHeight - 10;
            dancePos2 = maxLegHeight - 10;
        } else if (step==2){
            dancePos = maxLegHeight + 5;
            dancePos2 = maxLegHeight - 10;
        } else {
            dancePos = maxLegHeight + 5;
            dancePos2 = maxLegHeight + 5;
        }

        angles = {
            translate(1, 0, 0, dancePos, 0, 0, 0),
            translate(2, 0, 0, dancePos, 0, 0, 0),
            translate(3, 0, 0, dancePos2, 0, 0, 0),
            translate(4, 0, 0, dancePos2, 0, 0, 0),
        };
    }

    else {
        step = 0;
        dancePos = 0;
        dancing = false;
        angles = home();
    }

    return angles;
}

// Stable position to lay down when powered off
QuadJointAngles Kinematics::EmergencyShutdown() {
    QuadJointAngles angles = {
        translate(1, 5, -5, maxLegHeight - 20, 0, 0, 0),
        translate(2, 5, 5, maxLegHeight - 20, 0, 0, 0),
        translate(3, -5, 5, maxLegHeight - 20, 0, 0, 0),
        translate(4, -5, -5, maxLegHeight - 20, 0, 0, 0),
    };
    return angles;
}

// just returns the walk mode standing still
QuadJointAngles Kinematics::home() {
    QuadJointAngles angles = walk(0, 0, 0);
    return angles;
}
