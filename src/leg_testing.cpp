#include <Arduino.h>

#include "kinematics.hpp"
#include "config.hpp"
#include "log.hpp"
#include "utils.hpp"
#include "leg_testing.hpp"

// legs will be able to be tested individually when buttons are pressed
const int maxLegHeight = 380,
    minLegHeight = 320;

//if triangle is pressed, front right leg moves
//if square is pressed, front left leg moves
//if cross is pressed, back left leg moves
//if o is pressed, back right leg moves
QuadJointAngles Kinematics::legTesting(bool triangle_press, bool square_press, bool cross_press, bool o_press, float IMUpitch, float IMUroll){
    int legpos = maxLegHeight;
    int sitPos = maxLegHeight;

    float corrRoll = IMUroll * -0.3f; // correct for roll
    float corrPitch = IMUpitch * -0.3f; // correct for pitch

    QuadJointAngles angles = {};
    if(triangle_press){
        legpos = maxLegHeight - 40;
        angles = {
        translate (1, 0, 0, legpos, corrRoll, corrPitch, 0),
        
      };
    } else if(square_press){
        angles = {
        translate (2, 0, 0, legpos, corrRoll, corrPitch, 0),
        
      };
    } else if(cross_press){
        angles = {
        translate (3, 0, 0, legpos, corrRoll, corrPitch, 0),
        
      };
    } else if(o_press){
        angles = {
        translate (4, 0, 0, legpos, corrRoll, corrPitch, 0),
        
      };

    }
    return angles;
}