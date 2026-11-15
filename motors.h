#pragma once
#include <Arduino.h>

enum AxisId : uint8_t { AXIS_X, AXIS_Y, AXIS_A, AXIS_Z, AXIS_COUNT };
enum AxisState : uint8_t { AXIS_IDLE, AXIS_HOMING, AXIS_MOVING, AXIS_ERROR };

// ==================== MOTION SETTINGS ====================
const float DEFAULT_SPEED        = 4000;   // steps/s
const float DEFAULT_ACCELERATION = 2000;   // steps/s^2
const float HOMING_SPEED         = 4000;   // steps/s while searching the limit switch

const float MIN_SPEED        = 100;
const float MAX_SPEED_LIMIT  = 8000;       // an Uno loses steps well before this with 4 axes
const float MIN_ACCELERATION = 100;
const float MAX_ACCELERATION = 20000;

const long MAX_SEARCH_STEPS = 60000;       // homing aborts with error after this

// ==================== API ====================
void motorsInit();
void motorsUpdate();                       // call on every loop()
void motorsStop();                         // immediate stop, all axes need re-homing

void homeAxis(AxisId id);
bool moveAxis(AxisId id, long target);     // target in user steps, clamped to [0, max]

AxisState axisState(AxisId id);
long axisPosition(AxisId id);              // user coordinates
char axisName(AxisId id);

bool allHomed();
bool axesIdle();                           // no axis homing or moving
bool anyAxisError();

float setTravelSpeed(float stepsPerSecond);
float setAcceleration(float stepsPerSecond2);
void setZeroHere(AxisId id);   // declare current position as 0 without moving (no limit search)

void motorsPrintPositions();
void motorsPrintSettings();