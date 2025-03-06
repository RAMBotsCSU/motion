#include <Arduino.h>

#define THRESHOLD 10

int thresholdStick(int pos) {
    // threshold value for control sticks
    if (abs(pos) <= THRESHOLD) return 0;

    // SerialMon.printf("pos %d %d\n", pos, map(pos, -128, 128, -512, 512));

    return pos;
}

// motion filter to filter motions and compliance

float filter(float prevValue, float currentValue, int filter) {
    float lengthFiltered = (prevValue + (currentValue * filter)) / (filter + 1);
    return lengthFiltered;
}
