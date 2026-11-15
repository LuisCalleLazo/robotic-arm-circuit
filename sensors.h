#pragma once
#include <Arduino.h>

enum LimitId : uint8_t { LIMIT_X, LIMIT_Y, LIMIT_A, LIMIT_Z, LIMIT_COUNT };

// Level read when a switch is triggered (NC wiring + internal pull-up -> HIGH)
#define LIMIT_ACTIVE_LEVEL HIGH

void sensorsInit();
void sensorsUpdate();              // logs state changes, call every loop()
bool limitActive(LimitId id);
void sensorsPrintStatus();