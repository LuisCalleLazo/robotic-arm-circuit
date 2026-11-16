#include "z_axis.h"
#include "sensors.h"
#include <AccelStepper.h>

static AccelStepper zMotor(AccelStepper::DRIVER, STEP_Z_PIN, DIR_Z_PIN);
static bool referenced = false;
static long searchSteps = 0;

void zAxisInit() {
  pinMode(STEP_Z_PIN, OUTPUT);
  pinMode(DIR_Z_PIN, OUTPUT);
  digitalWrite(STEP_Z_PIN, LOW);
  zMotor.setMaxSpeed(MOVE_SPEED_Z);
  zMotor.setCurrentPosition(0);
}

bool zAxisBeginHoming() {
  referenced = false;
  searchSteps = 0;
  zMotor.setCurrentPosition(0);

  if (sensorsLimitActive(LIMIT_Z_PIN)) {
    referenced = true;
    return false;
  }

  zMotor.setSpeed(-MOVE_SPEED_Z);
  return true;
}

HomingStatus zAxisUpdateHoming() {
  if (sensorsLimitActive(LIMIT_Z_PIN)) {
    zMotor.setCurrentPosition(0);
    referenced = true;
    return HOMING_DONE;
  }

  if (zMotor.runSpeed()) searchSteps++;
  return HOMING_RUNNING;
}

long zAxisSearchSteps() {
  return searchSteps;
}

MoveStart zAxisMoveTo(long target) {
  if (!referenced) return MOVE_NOT_REFERENCED;

  target = constrain(target, 0L, MAX_TRAVEL_Z);
  zMotor.setMaxSpeed(MOVE_SPEED_Z);
  zMotor.moveTo(target);
  long distance = zMotor.distanceToGo();
  if (distance != 0) {
    zMotor.setSpeed(distance > 0 ? MOVE_SPEED_Z : -MOVE_SPEED_Z);
  }

  return distance == 0 ? MOVE_ALREADY_THERE : MOVE_STARTED;
}

MoveStatus zAxisRun() {
  long distance = zMotor.distanceToGo();
  if (distance == 0) return MOVE_DONE;

  if (distance < 0 && sensorsLimitActive(LIMIT_Z_PIN)) {
    zMotor.setCurrentPosition(0);
    return MOVE_LIMIT_HIT;
  }

  zMotor.runSpeed();
  return zMotor.distanceToGo() == 0 ? MOVE_DONE : MOVE_RUNNING;
}

long zAxisPosition() {
  return zMotor.currentPosition();
}

long zAxisTarget() {
  return zMotor.targetPosition();
}

bool zAxisIsReferenced() {
  return referenced;
}

void zAxisStop() {
  zMotor.setCurrentPosition(zMotor.currentPosition());
  referenced = false;
}
