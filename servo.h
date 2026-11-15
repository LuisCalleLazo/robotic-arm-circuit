#pragma once
#include <Arduino.h>

const int SERVO_OPEN_ANGLE   = 0;
const int SERVO_CLOSED_ANGLE = 100;

void servoInit();
bool servoAvailable();
bool servoMove(int angle);