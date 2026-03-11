#include <Arduino.h>
#include "odrive.hpp"
#include "log.hpp"

ODrive odrives[] = {
    ODrive(Serial1),
    ODrive(Serial2),
    ODrive(Serial3),
    ODrive(Serial4),
    ODrive(Serial5),
    ODrive(Serial6)
};

unsigned long lastLog = 0;
// Sets up serial connection, then connects to all ODrives
void setup() {
    Serial.begin(115200);
    while (!Serial) delay(10);

    Serial.println("=== ODrive Logger ===");

    for (int i = 0; i < 6; i++) {
        Serial.printf("Connecting ODrive %d...\n", i);
        odrives[i].connect();
    }
}

// Loop occurs every 1s to log ODrive axis states and errors to Serial
void loop() {
    unsigned long now = millis();
    if (now - lastLog >= 1000) {
        lastLog = now;
        
        // Checks and prints states and errors for all ODrives
        for (int i = 0; i < 6; i++) {
            int err0 = odrives[i].axis0.fetchError();
            int err1 = odrives[i].axis1.fetchError();
            int state0 = odrives[i].axis0.fetchState();
            int state1 = odrives[i].axis1.fetchState();

            Serial.printf("[ODrive %d] Axis0 state=%d err=%d | Axis1 state=%d err=%d\n",
                          i, state0, err0, state1, err1);
        }
    }
}
