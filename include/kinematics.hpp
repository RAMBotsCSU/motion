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
    QuadJointAngles walk(int RFB, int RLR, int LT,
                        float IMUpitch = 0.0f, float IMUroll = 0.0f);

    QuadJointAngles pushUp(bool cross_press, bool triangle_press,
                        float IMUpitch = 0.0f, float IMUroll = 0.0f);

    QuadJointAngles dance(bool up, bool down, bool left, bool right,
                        float IMUpitch = 0.0f, float IMUroll = 0.0f);
    QuadJointAngles Kinematics::EmergencyShutdown();


private:
    int lastRFB, lastRLR, lastLT;
    int step = 0;
    unsigned long lastStepAt;

    JointAngles translate(int leg, float xIn, float yIn, float zIn, float roll, float pitch, float yawIn);

};

#endif  // KINEMATICS_H
