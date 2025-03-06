#ifndef LEG_H
#define LEG_H

#include "odrive.h"
#include "kinematics.h"

class Leg {
public:
    Leg(int _id, Axis& _hip, Axis& _shoulder, Axis& _knee) : id(_id), hip(_hip), shoulder(_shoulder), knee(_knee) {}

    void move(JointAngles, float);

    String logPos(void);

private:
    const int id;
    Axis& hip;
    Axis& shoulder;
    Axis& knee;
};

#endif  // LEG_H
