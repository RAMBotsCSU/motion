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
    QuadJointAngles legControl(float left_stick_horizontal, float left_stick_vertical, 
        float left_trigger, float right_stick_horizontal, float right_stick_vertical, float right_trigger);
    QuadJointAngles EmergencyShutdown();
    QuadJointAngles legTesting(bool triangle_press, bool square_press, bool cross_press, bool o_press,
                        float IMUpitch = 0.0f, float IMUroll = 0.0f);
    
    float ema(float current, float target, float alpha);


private:
    int lastRFB, lastRLR, lastLT;
    int step = 0;
    float alpha = 0.1f;
    unsigned long lastStepAt;

    float left_legs_x;
    float left_legs_y;
    float left_legs_z = maxLegHeight;
    float right_legs_x;
    float right_legs_y;
    float right_legs_z = maxLegHeight;

    JointAngles translate(int leg, float xIn, float yIn, float zIn, float roll, float pitch, float yawIn);

};

#endif  // KINEMATICS_H
