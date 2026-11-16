#ifndef MOTORS_H
#define MOTORS_H

#include <Arduino.h>

// ==================== SPEED CONFIGURATION ====================
//
// HOMING_SPEED : steps/s while searching the limit switch (constant speed).
//
// Homing and normal moves use the same constant-speed step generator.
// Normal moves stop exactly at the requested position; there is no
// acceleration ramp. Keep the combined speed of simultaneously moving axes
// within the driver and controller limits.
const float HOMING_SPEED = 5000;
const float MOVE_SPEED_X = HOMING_SPEED;
const float MOVE_SPEED_Y = HOMING_SPEED;
const float MOVE_SPEED_A = HOMING_SPEED;
const float MOVE_SPEED_Z = HOMING_SPEED;

// STEP pulse width and DIR setup delay for the stepper drivers.
#define STEP_PULSE_US 2
#define DIR_SETUP_US 5

// Maximum steps searched for a limit switch before declaring a homing error.
const long HOMING_MAX_STEPS = 50000;

// ==================== DRIVER PINS ====================
#define STEP_X_PIN 2
#define DIR_X_PIN 5
#define STEP_Y_PIN 3
#define DIR_Y_PIN 6
#define STEP_A_PIN 4
#define DIR_A_PIN 7
#define STEP_Z_PIN 12
#define DIR_Z_PIN 13

// ==================== TRAVEL LIMITS (logical steps from zero) ====================
const long MAX_TRAVEL_X = 30450;
const long MAX_TRAVEL_Y = 11834;
const long MAX_TRAVEL_A = 3426;
const long MAX_TRAVEL_Z = 47829;

// ==================== TYPES ====================
enum AxisId : uint8_t { AXIS_X = 0, AXIS_Y, AXIS_A, AXIS_Z, AXIS_COUNT };

// Result of motorsHomingUpdate().
enum HomingStatus : uint8_t { HOMING_RUNNING, HOMING_DONE, HOMING_FAILED };

// Result of motorsMoveTo().
enum MoveStart : uint8_t { MOVE_STARTED, MOVE_ALREADY_THERE, MOVE_NOT_REFERENCED };

// Result of motorsRunAxis().
enum MoveStatus : uint8_t { MOVE_RUNNING, MOVE_DONE, MOVE_LIMIT_HIT };

// ==================== API ====================
// Positions are logical: 0 is the limit switch and positive moves away from it.

void motorsInit();

// Axis lookup helpers.
char motorsName(AxisId id);
bool motorsAxisFromChar(char c, AxisId *id);

// Call motorsHomingStart() once, then motorsHomingUpdate() each loop until it
// returns HOMING_DONE or HOMING_FAILED.
// motorsHomingStart() returns false if the limit was already active
// (the axis is referenced immediately and no search is needed).
bool motorsHomingStart(AxisId id);
HomingStatus motorsHomingUpdate(AxisId id);
long motorsSearchSteps(AxisId id);

// Sets the target of one axis (clamped to its travel). Does not block.
MoveStart motorsMoveTo(AxisId id, long target);

// Generates steps of one axis.
MoveStatus motorsRunAxis(AxisId id);

// Generates steps of all axes. Returns true while any axis is moving.
bool motorsRunAll();

// State queries.
long motorsPosition(AxisId id);
long motorsTarget(AxisId id);
bool motorsIsReferenced(AxisId id);
bool motorsAllReferenced();

// Stops all axes immediately and clears their references.
void motorsStopAll();

#endif