#include <Arduino.h>
#include "MPU6050.h"

#define CALIBRATION_TIME_MS 5000  // Collect samples for 5 seconds
#define SAMPLE_DELAY_MS 10

MPU6050 mpu;

float accelBiasX = 0.0f, accelBiasY = 0.0f, accelBiasZ = 0.0f;
float gyroBiasX = 0.0f, gyroBiasY = 0.0f, gyroBiasZ = 0.0f;

unsigned long startTime;
unsigned long sampleCount = 0;

void setup() {
    Serial.begin(115200);
    while (!Serial) delay(10);

    Serial.println("=== IMU Calibration Tool ===");

    if (!mpu.begin()) {
        Serial.println("Failed to initialize IMU!");
        while (1) delay(100);
    }

    mpu.setAccelerometerRange(MPU6050_RANGE_16_G);
    mpu.setGyroRange(MPU6050_RANGE_250_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

    Serial.println("Keep the robot completely still on a level surface...");
    delay(2000);

    startTime = millis();
}

void loop() {
    if (millis() - startTime < CALIBRATION_TIME_MS) {
        sensors_event_t a, g, temp;
        mpu.getEvent(&a, &g, &temp);

        accelBiasX += a.acceleration.x;
        accelBiasY += a.acceleration.y;
        accelBiasZ += a.acceleration.z;
        gyroBiasX += g.gyro.x;
        gyroBiasY += g.gyro.y;
        gyroBiasZ += g.gyro.z;

        sampleCount++;
        delay(SAMPLE_DELAY_MS);
    } else {
        // Done collecting samples
        accelBiasX /= sampleCount;
        accelBiasY /= sampleCount;
        accelBiasZ /= sampleCount;
        gyroBiasX /= sampleCount;
        gyroBiasY /= sampleCount;
        gyroBiasZ /= sampleCount;

        Serial.println("\n=== Calibration Complete ===");
        Serial.printf("Accel bias (m/s^2): X=%.4f Y=%.4f Z=%.4f\n",
                      accelBiasX, accelBiasY, accelBiasZ);
        Serial.printf("Gyro bias (°/s):    X=%.4f Y=%.4f Z=%.4f\n",
                      gyroBiasX, gyroBiasY, gyroBiasZ);

        Serial.println("\nSuggested constants:");
        Serial.printf("const float pitchTrim = %.2ff;\n", -atan2(accelBiasY, accelBiasZ) * 57.2958);
        Serial.printf("const float rollTrim  = %.2ff;\n", atan2(accelBiasX, accelBiasZ) * 57.2958);

        while (1) delay(1000);
    }
}
