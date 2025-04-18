#include <Arduino.h>
#include "sparky.hpp"
#include "log.hpp"

Sparky sparky;


// ****************** SETUP ******************************
void setup() {
    // logging
    SerialMon.begin(115200);

    if (CrashReport) SerialMon.println(CrashReport);
    Log("init");

    sparky.setup();
}


// ********************* MAIN LOOP *******************************
void loop() {
    sparky.update();
}
