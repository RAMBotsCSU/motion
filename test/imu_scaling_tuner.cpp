#include <Arduino.h>
#include "sparky.hpp"
#include "config.hpp"
#include "MPU6050.h"

#define UPDATE_MS 50  // main loop period

Sparky sparky;
MPU6050 mpu;

// Correction scaling (adjustable at runtime)
float pitchScale = 0.3f;
float rollScale  = 0.3f;

// Timing variables
unsigned long lastUpdate = 0;
unsigned long lastPrint = 0;

void setup() {
    Serial.begin(115200);
    while (!Serial) delay(10);

    // Basic keyboard UI to adjust scaling factors
    Serial.println("\n=== IMU Scaling Tuner ===");
    Serial.println("Controls:");
    Serial.println("  p/P - increase/decrease pitch scale");
    Serial.println("  r/R - increase/decrease roll scale");
    Serial.println("  h   - print current values\n");

    sparky.setup();

    // Checks if IMU initialized successfully, then halts the program if it has not
    if (!mpu.begin()) {
        Serial.println("IMU failed to initialize!");
        while (1) delay(100);
    }

    mpu.setAccelerometerRange(MPU6050_RANGE_16_G);
    mpu.setGyroRange(MPU6050_RANGE_250_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

    Serial.println("IMU ready. Robot entering home position...");
}

void loop() {
    unsigned long now = millis();

    // === handle serial input (basic UI) ===
    if (Serial.available()) {
        char c = Serial.read();
        switch (c) {
            case 'p': pitchScale += 0.05f; break;
            case 'P': pitchScale = max(0.0f, pitchScale - 0.05f); break;
            case 'r': rollScale += 0.05f; break;
            case 'R': rollScale = max(0.0f, rollScale - 0.05f); break;
            case 'h':
                Serial.printf("Pitch Scale: %.3f | Roll Scale: %.3f\n", pitchScale, rollScale);
                break;
            default: break;
        }
    }

    // === update every UPDATE_MS ===
    if (now - lastUpdate >= UPDATE_MS) {
        lastUpdate = now;

        sensors_event_t a, g, temp;
        mpu.getEvent(&a, &g, &temp);

        // Calculate accelerometer tilt
        float pitch = atan2(a.acceleration.y, a.acceleration.z) * 57.2958f;
        float roll  = atan2(a.acceleration.x, a.acceleration.z) * 57.2958f;

        // Use same trims as sparky.cpp
        const float pitchTrim = 2.7f;
        const float rollTrim  = -5.0f;

        float correctedPitch = (pitch + pitchTrim) * pitchScale;
        float correctedRoll  = (roll + rollTrim) * rollScale;

        // Get base home pose from kinematics
        QuadJointAngles angles = sparky.kinematics.home();

        // Apply IMU correction (feedforward)
        angles.FR.hip      += correctedRoll;
        angles.FL.hip      -= correctedRoll;
        angles.BR.hip      += correctedRoll;
        angles.BL.hip      -= correctedRoll;

        angles.FR.shoulder += correctedPitch;
        angles.FL.shoulder += correctedPitch;
        angles.BR.shoulder += correctedPitch;
        angles.BL.shoulder += correctedPitch;

        // Move all legs to new angles
        sparky.leg[0].move(angles.FR);
        sparky.leg[1].move(angles.FL);
        sparky.leg[2].move(angles.BL);
        sparky.leg[3].move(angles.BR);
    }

    // === periodic printout ===
    if (now - lastPrint >= 1000) {
        lastPrint = now;
        Serial.printf("PitchScale=%.2f  RollScale=%.2f\n", pitchScale, rollScale);
    }
}
