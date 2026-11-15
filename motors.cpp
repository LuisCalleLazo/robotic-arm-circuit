#include "motors.h"
#include "sensors.h"
#include <AccelStepper.h>

// ==================== PINS ====================
#define STEP_X 2
#define DIR_X  5
#define STEP_Y 3
#define DIR_Y  6
#define STEP_A 4
#define DIR_A  7
#define STEP_Z 12
#define DIR_Z  13

static AccelStepper stepperX(AccelStepper::DRIVER, STEP_X, DIR_X);
static AccelStepper stepperY(AccelStepper::DRIVER, STEP_Y, DIR_Y);
static AccelStepper stepperA(AccelStepper::DRIVER, STEP_A, DIR_A);
static AccelStepper stepperZ(AccelStepper::DRIVER, STEP_Z, DIR_Z);

// ==================== AXIS TABLE ====================
struct Axis {
  char name;
  AccelStepper* stepper;
  LimitId limit;
  long maxPos;        // software limit (user steps)
  int8_t sign;        // user position = sign * stepper position
  AxisState state;
  bool homed;
  long searchSteps;
};

// Homing direction in stepper domain is always -sign.
static Axis axes[AXIS_COUNT] = {
  { 'X', &stepperX, LIMIT_X, 30450L, -1, AXIS_IDLE, false, 0 },
  { 'Y', &stepperY, LIMIT_Y, 11834L, -1, AXIS_IDLE, false, 0 },
  { 'A', &stepperA, LIMIT_A,  3426L, +1, AXIS_IDLE, false, 0 },
  { 'Z', &stepperZ, LIMIT_Z, 47829L, +1, AXIS_IDLE, false, 0 },
};

static float travelSpeed  = DEFAULT_SPEED;
static float acceleration = DEFAULT_ACCELERATION;

// ==================== HELPERS ====================
static void logAxis(const Axis& ax, const __FlashStringHelper* msg) {
  Serial.print(F("[MOTOR] ")); Serial.print(ax.name);
  Serial.print(F(": ")); Serial.println(msg);
}

static void logAxisValue(const Axis& ax, const __FlashStringHelper* msg, long value) {
  Serial.print(F("[MOTOR] ")); Serial.print(ax.name);
  Serial.print(F(": ")); Serial.print(msg); Serial.println(value);
}

static void applyMotion(Axis& ax) {
  ax.stepper->setMaxSpeed(travelSpeed);
  ax.stepper->setAcceleration(acceleration);
}

static bool movingTowardHome(const Axis& ax) {
  long dist = ax.stepper->distanceToGo();
  return (ax.sign < 0) ? (dist > 0) : (dist < 0);
}

// ==================== HOMING ====================
static void finishHoming(Axis& ax) {
  ax.stepper->setCurrentPosition(0);
  applyMotion(ax);                       // restore travel speed after homing
  ax.homed = true;
  ax.state = AXIS_IDLE;
  logAxisValue(ax, F("limit reached, zero set. Search steps = "), ax.searchSteps);
}

void homeAxis(AxisId id) {
  Axis& ax = axes[id];
  ax.homed = false;
  ax.searchSteps = 0;
  ax.stepper->setCurrentPosition(0);

  if (limitActive(ax.limit)) {
    applyMotion(ax);
    ax.homed = true;
    ax.state = AXIS_IDLE;
    logAxis(ax, F("limit already active, zero set"));
    return;
  }

  ax.stepper->setMaxSpeed(HOMING_SPEED);
  ax.stepper->setSpeed(-ax.sign * HOMING_SPEED);
  ax.state = AXIS_HOMING;
  logAxis(ax, F("homing: searching limit switch..."));
}

// ==================== MOVEMENT ====================
bool moveAxis(AxisId id, long target) {
  Axis& ax = axes[id];

  if (!ax.homed) {
    logAxis(ax, F("not homed, move rejected"));
    return false;
  }

  long requested = target;
  target = constrain(target, 0L, ax.maxPos);
  if (target != requested) logAxisValue(ax, F("target clamped to "), target);

  ax.stepper->moveTo(ax.sign * target);

  if (ax.stepper->distanceToGo() == 0) {
    ax.state = AXIS_IDLE;
    logAxisValue(ax, F("already at "), target);
  } else {
    ax.state = AXIS_MOVING;
    logAxisValue(ax, F("moving to "), target);
  }
  return true;
}

// ==================== UPDATE ====================
static void updateAxis(Axis& ax) {
  switch (ax.state) {
    case AXIS_HOMING:
      if (limitActive(ax.limit)) {
        finishHoming(ax);
      } else if (ax.searchSteps >= MAX_SEARCH_STEPS) {
        ax.state = AXIS_ERROR;
        ax.stepper->setSpeed(0);
        logAxisValue(ax, F("ERROR: limit not found after steps = "), ax.searchSteps);
      } else if (ax.stepper->runSpeed()) {
        ax.searchSteps++;
      }
      break;

    case AXIS_MOVING:
      if (movingTowardHome(ax) && limitActive(ax.limit)) {
        ax.stepper->setCurrentPosition(0);
        ax.state = AXIS_IDLE;
        logAxis(ax, F("limit hit while moving, stopped and re-zeroed"));
      } else {
        ax.stepper->run();
        if (ax.stepper->distanceToGo() == 0) {
          ax.state = AXIS_IDLE;
          logAxisValue(ax, F("arrived at "), axisPosition((AxisId)(&ax - axes)));
        }
      }
      break;

    default:
      break;
  }
}

void motorsUpdate() {
  for (uint8_t i = 0; i < AXIS_COUNT; i++) updateAxis(axes[i]);
}

void motorsStop() {
  for (uint8_t i = 0; i < AXIS_COUNT; i++) {
    Axis& ax = axes[i];
    ax.stepper->setCurrentPosition(ax.stepper->currentPosition());  // clears target and speed
    applyMotion(ax);
    ax.state = AXIS_IDLE;
    if (i != AXIS_Z) ax.homed = false;   // Z keeps its tracked position (it has no homing)
  }
  Serial.println(F("[MOTOR] All axes stopped, X/Y/A need re-homing"));
}

// ==================== SETTINGS ====================
float setTravelSpeed(float stepsPerSecond) {
  travelSpeed = constrain(stepsPerSecond, MIN_SPEED, MAX_SPEED_LIMIT);
  for (uint8_t i = 0; i < AXIS_COUNT; i++) {
    if (axes[i].state != AXIS_HOMING) applyMotion(axes[i]);
  }
  return travelSpeed;
}

float setAcceleration(float stepsPerSecond2) {
  acceleration = constrain(stepsPerSecond2, MIN_ACCELERATION, MAX_ACCELERATION);
  for (uint8_t i = 0; i < AXIS_COUNT; i++) {
    if (axes[i].state != AXIS_HOMING) applyMotion(axes[i]);
  }
  return acceleration;
}

void motorsInit() {
  for (uint8_t i = 0; i < AXIS_COUNT; i++) applyMotion(axes[i]);
  motorsPrintSettings();
}

// ==================== QUERIES ====================
AxisState axisState(AxisId id) { return axes[id].state; }
char axisName(AxisId id) { return axes[id].name; }

long axisPosition(AxisId id) {
  return axes[id].sign * axes[id].stepper->currentPosition();
}

bool allHomed() {
  for (uint8_t i = 0; i < AXIS_COUNT; i++) if (!axes[i].homed) return false;
  return true;
}

bool axesIdle() {
  for (uint8_t i = 0; i < AXIS_COUNT; i++) {
    if (axes[i].state == AXIS_HOMING || axes[i].state == AXIS_MOVING) return false;
  }
  return true;
}

bool anyAxisError() {
  for (uint8_t i = 0; i < AXIS_COUNT; i++) if (axes[i].state == AXIS_ERROR) return true;
  return false;
}

// ==================== PRINTING ====================
void motorsPrintPositions() {
  for (uint8_t i = 0; i < AXIS_COUNT; i++) {
    Serial.print(axes[i].name);
    Serial.print('=');
    Serial.print(axisPosition((AxisId)i));
    if (i < AXIS_COUNT - 1) Serial.print(' ');
  }
  Serial.println();
}

void motorsPrintSettings() {
  Serial.print(F("[MOTOR] speed="));
  Serial.print(travelSpeed, 0);
  Serial.print(F(" steps/s | acc="));
  Serial.print(acceleration, 0);
  Serial.print(F(" steps/s2 | homed: "));
  bool any = false;
  for (uint8_t i = 0; i < AXIS_COUNT; i++) {
    if (axes[i].homed) { Serial.print(axes[i].name); any = true; }
  }
  if (!any) Serial.print('-');
  Serial.println();
}

void setZeroHere(AxisId id) {
  Axis& ax = axes[id];
  ax.stepper->setCurrentPosition(0);
  applyMotion(ax);
  ax.homed = true;
  ax.state = AXIS_IDLE;
  logAxis(ax, F("zero set at current position (no limit search)"));
}