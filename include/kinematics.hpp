#ifndef KINEMATICS_H
#define KINEMATICS_H

struct JointAngles {
    float shoulder;
    float knee;
    float hip;
};

struct QuadJointAngles {
    JointAngles FR;
    JointAngles FL;
    JointAngles BL;
    JointAngles BR;
};


class Kinematics {
public:
    void reset(void);

    QuadJointAngles home();
    QuadJointAngles walk(int RFB, int RLR, int LT);
    QuadJointAngles pushUp(bool);
    QuadJointAngles dance(bool, bool, bool, bool);


private:
    int lastRFB, lastRLR, lastLT;
    int step = 0;
    unsigned long lastStepAt;

    JointAngles translate(int leg, float xIn, float yIn, float zIn, float roll, float pitch, float yawIn);

};

#endif  // KINEMATICS_H
