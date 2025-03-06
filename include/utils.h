#ifndef UTILS_H
#define UTILS_H

#include <Arduino.h>

int thresholdStick(int pos);

float filter(float prevValue, float currentValue, int filter);

#endif  // UTILS_H
