#include <Arduino.h>
#include "sparky.hpp"
#include "odrive.hpp"
#include "MPU6050.h"

Sparky sparky;
MPU6050 mpu;

unsigned long lastLog = 0;

void setup() {
    Serial.begin(115200);
    while (!Serial) delay(10);

    Serial.println("=== System Health Monitor ===");

    sparky.setup();
    
    // Checks if IMU initialized successfully, then halts the program if it has not
    if (!mpu.begin()) {
        Serial.println("IMU initialization failed!");
        while (1);
    }
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
}

// Loop occurs every 0.5s to log IMU data and ODrive errors to Serial
void loop() {
    unsigned long now = millis();
    if (now - lastLog >= 500) {
        lastLog = now;

        sensors_event_t a, g, temp;
        mpu.getEvent(&a, &g, &temp);

        Serial.printf("[IMU] PitchRoll raw X=%.2f Y=%.2f Z=%.2f | Gyro X=%.2f Y=%.2f\n",
            a.acceleration.x, a.acceleration.y, a.acceleration.z,
            g.gyro.x, g.gyro.y);

        for (int i = 0; i < 6; i++) {
            int err0 = sparky.odrive[i].axis0.fetchError();
            int err1 = sparky.odrive[i].axis1.fetchError();
            Serial.printf("[ODrive %d] Err0=%d Err1=%d\n", i, err0, err1);
        }

        Serial.println("-----------------------------");
    }
}
