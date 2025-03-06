#include "leg.h"

void Leg::move(JointAngles angles, float duration) {
    hip.move(angles.hip, duration);
    shoulder.move(angles.shoulder, duration);
    knee.move(angles.knee, duration);
}

String Leg::logPos() {
    return "Hip: " + String(hip.getCurrentPos()) +
           " Shoulder: " + String(shoulder.getCurrentPos()) +
           " Knee: " + String(knee.getCurrentPos());
}
