#include "leg.hpp"

void Leg::move(JointAngles angles) {
    hip.move(angles.hip);
    shoulder.move(angles.shoulder);
    knee.move(angles.knee);
}
