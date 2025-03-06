#include "odrive.hpp"
#include "log.hpp"

bool ODrive::init() {
    // ensure serial is clear
    // serial.write("\n\n");
    // serial.read();
    serial.flush();

    // check if connected
    serial.write("r serial_number\n");
    String sn = serial.readStringUntil('\n');

    if(sn.length() == 0) {
        Log("Failed to connect to odrive\n");
        return false;
    }

    Log("Connected to SN: %s\n", sn.substring(0, sn.length() - 1).c_str());

    // init axis
    axis0.init();
    axis1.init();

    return true;
}
