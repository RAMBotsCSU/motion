# Motion/src

## High-Level Code Overview

### axis.cpp

Contains methods for initializing axis, fetching coordinates, and resetting ODrives.
Contains method for sending position to ODrives.

### kinematics.cpp

Contains method for inverse kinematics calculations, which takes position (x, y, z, roll, pitch, yaw), and translates them to relative ODrive movements for each joint.
Contains methods for walking, pushups, and dancing.

### leg.cpp

Contains helper method for movement of all joints in a leg.

### log.cpp

Contains helper method for logging errors/information while DEBUG flag is set. Running in debug mode slows serial communications for the rest of the robot.

### main.cpp

Initializes serial communications of the robot, then calls setup method in sparky.cpp.
Contains loop code, which calls update method in sparky.cpp.

### odrive.cpp

Contains methods for initializing and connecting to an ODrive.
Initializes axis for each ODrive.
Defines parameters for ODrive setup.

### sparky.cpp

Contains setup method, which initializes all serial communications on the robot. 
Contains update method, which is called in the main.cpp loop. Update checks tick time so that the robot is updated every 10ms, then checks which mode (walk, pushup, dance) the robot is in. IMU data is sent to the respective kinematics function. The IMU is then updated with current values. Updates to internal states are then made (button presses, operation mode).

### utils.cpp

Contains methods for trimming stick positions (controller deadzone) and filtering motions.

### High Level Summary

Upon running setup in main.cpp, the following happen:
* A serial monitor is set up
* sparky.setup() is called
* In sparky.setup(), serial connections are created for all 6 of the ODrives. The IMU is then initialized, and all ODrive connections are made

The majority of functionality occurs in sparky.update(), which is called from main.loop(). The following happen:
* The current time is fetched. If it has been < 10ms since the last update, sparky does not update
* Ensures controller and ODrives are connected
* Checks current mode, then calls the respective method in kinematics.cpp (walk, pushUp, or dance)
* Checks the IMU for current roll/pitch, and stores them in class variables
* Checks for updates to controller state, and stores button values in class variables
* Checks for ODrive errors

## Low-Level Code Overview

### axis.cpp

Axis::Axis(Odrive& _odrive, int _id):
* Sets ODrive and id of axis to specified parameters (there are 2 Odrives per axis)

Axis::init():
* Checks for errors then tries to reset them by calling Axis::reset()
* Updates zero position offsets
* Sets the axis to closed loop by calling Axis::setClosedLoop()
* Calls Axis::fetchState() and logs the state of the axis
* Sends axis config to ODrives on each axis
* Checks if ODrive response has a nonzero length
* If the response length is nonzero, sending the axis config failed
* ODrive response is logged if setting the axis config failed

Axis::fetchOffset():
* Gets the position estimate of the odrive, converts it to a float, and stores it in offset

Axis::reset():
* Attempts to reset the ODrive by setting controller, enoder, motor, and general error flags to 0
* Sends "sc\n" to the ODrive, clearing communications (I presume this is short for serial clear, but I was unable to find it in the ODrive documentation)

Axis::setClosedLoop():
* Sends data to the ODrive, requesting closed loop state

Axis::fetchState():
* Returns an integer state of the ODrive's control mode

Axis::move(float pos):
* Checks if provided position is different from target position
* If positions are different, sets target position to pos, then sends targetPos + offset to the ODrive

Axis::getOffset():


Axis::fetchError():


Axis::setSpeed(float speed):


### kinematics.cpp

### leg.cpp

### log.cpp

### main.cpp

### odrive.cpp

### sparky.cpp

### utils.cpp