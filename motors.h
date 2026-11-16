#ifndef MOTORS_H
#define MOTORS_H

#include <Arduino.h>

// ==================== SPEED CONFIGURATION ====================
// This is the ONLY place where speeds and accelerations are configured.
//
// HOMING_SPEED : steps/s while searching the limit switch (constant speed).
//
// MOVE_SPEED_x : maximum steps/s of axis x during normal moves.
// MOVE_ACCEL_x : steps/s^2 used to ramp up and down on axis x.
//                Higher acceleration = reaches the top speed sooner and
//                short moves (like A) also get fast. Lower = softer start.
//
// X, Y and A use the lightweight custom ramp below. Z uses AccelStepper at
// constant speed for both homing and normal travel.
//
// Custom-ramp movement:
//  - Steps are generated at a constant rate between ramp updates, exactly
//    like the homing search (cheap, no floating point per step).
//  - The speed ramp (trapezoid) is recomputed only every RAMP_UPDATE_US,
//    not on every step. This keeps the main loop fast while the other axes
//    run alongside Z.
//
// Tuning guide:
//  1. Raise MOVE_SPEED_x by 1000 at a time until an axis starts losing
//     steps or makes noise, then go back one step.
//  2. If an axis stalls when starting, lower its MOVE_ACCEL_x.
//  3. CPU limit: each step costs about 20-30 us on an Arduino Uno/Nano. The
//     sum of the speeds of the axes that move AT THE SAME TIME should stay
//     below roughly 25000-30000 steps/s. Moving a single axis can go higher.
const float HOMING_SPEED = 5000;

const float MOVE_SPEED_X = 6000;
const float MOVE_ACCEL_X = 3000;
const float MOVE_SPEED_Y = 6000;
const float MOVE_ACCEL_Y = 15000;
const float MOVE_SPEED_A = 6000;
const float MOVE_ACCEL_A = 15000;
const float MOVE_SPEED_Z = HOMING_SPEED;

// Ramp internals (normally you do not need to touch these).
// RAMP_UPDATE_US  : how often the speed of every axis is recomputed.
// RAMP_START_SPEED: steps/s at the start and end of a move (minimum speed).
// STEP_PULSE_US   : width of the STEP pulse. Increase to 3-5 if your driver
//                   (for example DRV8825) misses steps.
// DIR_SETUP_US    : wait after changing the DIR pin before stepping.
const unsigned long RAMP_UPDATE_US = 2000;
const float RAMP_START_SPEED = 300;
#define STEP_PULSE_US 2
#define DIR_SETUP_US 5

// Maximum steps allowed while searching a limit switch before failing.
// As in the original code, the Z axis does not use this limit.
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
// Generic movement functions control X, Y and A. Z is controlled by z_axis.h.
// Positions are logical: 0 is the limit switch and positive moves away from it.

void motorsInit();

// Axis lookup helpers.
char motorsName(AxisId id);
bool motorsAxisFromChar(char c, AxisId *id);

// X/Y/A homing. Z homing is provided by zAxisBeginHoming()/zAxisUpdateHoming().
// Call motorsHomingStart() once, then motorsHomingUpdate() each loop until it
// returns HOMING_DONE or HOMING_FAILED.
// motorsHomingStart() returns false if the limit was already active
// (the axis is referenced immediately and no search is needed).
bool motorsHomingStart(AxisId id);
HomingStatus motorsHomingUpdate(AxisId id);
long motorsSearchSteps(AxisId id);

// Sets the target of one X/Y/A axis (clamped to its travel). Does not block.
MoveStart motorsMoveTo(AxisId id, long target);

// Generates steps of one X/Y/A axis.
MoveStatus motorsRunAxis(AxisId id);

// Generates steps of X/Y/A. Z must be advanced separately with zAxisRun().
bool motorsRunAll();

// State queries for X/Y/A.
long motorsPosition(AxisId id);
long motorsTarget(AxisId id);
bool motorsIsReferenced(AxisId id);
bool motorsAllReferenced();

// Stops X/Y/A immediately and clears their references.
void motorsStopAll();

#endif