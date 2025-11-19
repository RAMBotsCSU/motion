#include <Arduino.h>
#include "MPU6050.h"

#define LOG_INTERVAL 100  // ms

MPU6050 mpu;

unsigned long lastLog = 0;

void setup() {
    Serial.begin(115200);
    while (!Serial) delay(10);

    Serial.println("=== IMU Logger ===");

    if (!mpu.begin()) {
        Serial.println("Failed to initialize IMU!");
        while (1) delay(100);
    }

    mpu.setAccelerometerRange(MPU6050_RANGE_16_G);
    mpu.setGyroRange(MPU6050_RANGE_250_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
}

void loop() {
    unsigned long now = millis();
    if (now - lastLog >= LOG_INTERVAL) {
        lastLog = now;

        sensors_event_t a, g, temp;
        mpu.getEvent(&a, &g, &temp);

        Serial.printf("[IMU] Accel (m/s^2): X=%.2f Y=%.2f Z=%.2f | Gyro (°/s): X=%.2f Y=%.2f Z=%.2f | Temp: %.2f°C\n",
            a.acceleration.x, a.acceleration.y, a.acceleration.z,
            g.gyro.x, g.gyro.y, g.gyro.z, temp.temperature);
    }
}
