#ifndef SERVO_H
#define SERVO_H

#include <Arduino.h>

// ==================== PCA9685 (SOFTWARE I2C) ====================
#define SERVO_SCL_PIN A0
#define SERVO_SDA_PIN A1
#define PCA9685_ADDR 0x40
#define SERVO_CHANNEL 0

// Gripper angles. Adjust here to change how far the servo opens or closes.
const int SERVO_OPEN_ANGLE = 0;
const int SERVO_CLOSED_ANGLE = 140;

// Detects and configures the PCA9685. Returns true on success.
bool servoInit();

// True if the PCA9685 was detected during servoInit().
bool servoIsAvailable();

// Moves the servo to `angle` (0..180). Returns false if the board is
// missing or an I2C transfer was not acknowledged.
bool servoMove(int angle);

#endif