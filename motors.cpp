#include "motors.h"
#include "sensors.h"

// Homing and normal moves share the same constant-speed step generator.
//
// sign: converts logical position to motor steps (motorSteps = sign * logical).
//       -1 inverts the physical direction. Homing always moves toward the
//       limit switch, i.e. toward logical negative.
// stopOnLimit: if true, a normal move toward zero is cancelled when the
//       limit switch activates (used by Z as a crash protection).
// limitedSearch: if true, homing fails after HOMING_MAX_STEPS.
struct Axis {
  // ----- fixed configuration -----
  uint8_t stepPin, dirPin, limitPin;
  char name;
  int8_t sign;
  long maxTravel;
  float maxSpeed;      // steps/s
  bool stopOnLimit;
  bool limitedSearch;

  // ----- runtime state (motor domain: motor steps, not logical) -----
  bool referenced;
  bool moving;         // true while a constant-speed move is in progress
  long searchSteps;
  long pos;            // current position in motor steps
  long target;         // target position in motor steps
  unsigned long intervalUs;   // time between steps, 0 = not stepping
  unsigned long lastStepUs;   // time of the last step
  int8_t stepDir;      // direction of the next steps: +1 or -1
  int8_t dirLevel;     // level currently on the DIR pin: +1, -1, 0 = unknown

  Axis(uint8_t sp, uint8_t dp, uint8_t lp, char n, int8_t s, long travel,
       float vmax, bool stopLimit, bool limitSearch)
    : stepPin(sp), dirPin(dp), limitPin(lp), name(n), sign(s),
      maxTravel(travel), maxSpeed(vmax),
      stopOnLimit(stopLimit), limitedSearch(limitSearch),
      referenced(false), moving(false), searchSteps(0), pos(0), target(0),
      intervalUs(0), lastStepUs(0), stepDir(1), dirLevel(0) {}
};

//                 step pin    dir pin    limit pin    name sign travel        speed        stopOnLimit limitedSearch
static Axis axes[AXIS_COUNT] = {
  Axis(STEP_X_PIN, DIR_X_PIN, LIMIT_X_PIN, 'X', -1, MAX_TRAVEL_X, MOVE_SPEED_X, false, true),
  Axis(STEP_Y_PIN, DIR_Y_PIN, LIMIT_Y_PIN, 'Y', -1, MAX_TRAVEL_Y, MOVE_SPEED_Y, false, true),
  Axis(STEP_A_PIN, DIR_A_PIN, LIMIT_A_PIN, 'A',  1, MAX_TRAVEL_A, MOVE_SPEED_A, false, true),
  Axis(STEP_Z_PIN, DIR_Z_PIN, LIMIT_Z_PIN, 'Z',  1, MAX_TRAVEL_Z, MOVE_SPEED_Z, true, true)
};

// ==================== LOW LEVEL STEP GENERATION ====================
// Sends one STEP pulse in direction a.stepDir. The DIR pin is only written
// when the direction changes (saves time on every step).
static void doStep(Axis &a) {
  if (a.dirLevel != a.stepDir) {
    digitalWrite(a.dirPin, a.stepDir > 0 ? HIGH : LOW);
    a.dirLevel = a.stepDir;
    delayMicroseconds(DIR_SETUP_US);
  }
  digitalWrite(a.stepPin, HIGH);
  delayMicroseconds(STEP_PULSE_US);
  digitalWrite(a.stepPin, LOW);
  a.pos += a.stepDir;
}

// Sends a step if the interval since the previous one has elapsed.
// Returns true if a step was sent. Uses "last += interval" so the average
// rate is exact; if the loop was late by more than one interval it
// resynchronizes instead of sending a burst of steps.
static bool stepIfDue(Axis &a, unsigned long now) {
  if (a.intervalUs == 0) return false;
  if (now - a.lastStepUs < a.intervalUs) return false;
  a.lastStepUs += a.intervalUs;
  if (now - a.lastStepUs >= a.intervalUs) a.lastStepUs = now;
  doStep(a);
  return true;
}

static void clearMotion(Axis &a) {
  a.moving = false;
  a.intervalUs = 0;
}

// Sends the due step of one moving axis. Returns true while it is moving.
static bool stepAxisMove(Axis &a, unsigned long now) {
  if (!a.moving) return false;

  long remaining = a.target - a.pos;
  if (remaining == 0) { clearMotion(a); return false; }

  stepIfDue(a, now);
  return true;
}

// ==================== INIT AND LOOKUP ====================
void motorsInit() {
  for (uint8_t i = 0; i < AXIS_COUNT; i++) {
    pinMode(axes[i].stepPin, OUTPUT);
    pinMode(axes[i].dirPin, OUTPUT);
    digitalWrite(axes[i].stepPin, LOW);
  }
}

char motorsName(AxisId id) {
  return axes[id].name;
}

bool motorsAxisFromChar(char c, AxisId *id) {
  for (uint8_t i = 0; i < AXIS_COUNT; i++) {
    if (axes[i].name == c) {
      *id = (AxisId)i;
      return true;
    }
  }
  return false;
}

// ==================== HOMING ====================
bool motorsHomingStart(AxisId id) {
  Axis &a = axes[id];
  clearMotion(a);
  a.referenced = false;
  a.searchSteps = 0;
  a.pos = 0;
  a.target = 0;
  if (sensorsLimitActive(a.limitPin)) {
    a.referenced = true;
    return false;
  }

  // Homing and normal moves use the same constant-speed step interval.
  a.stepDir = -a.sign;
  a.intervalUs = (unsigned long)(1000000.0f / HOMING_SPEED);
  a.lastStepUs = micros();
  return true;
}

HomingStatus motorsHomingUpdate(AxisId id) {
  Axis &a = axes[id];

  if (sensorsLimitActive(a.limitPin)) {
    clearMotion(a);
    a.pos = 0;
    a.target = 0;
    a.referenced = true;
    return HOMING_DONE;
  }
  if (a.limitedSearch && a.searchSteps >= HOMING_MAX_STEPS) {
    clearMotion(a);
    return HOMING_FAILED;
  }
  if (stepIfDue(a, micros())) {
    a.searchSteps++;
  }
  return HOMING_RUNNING;
}

long motorsSearchSteps(AxisId id) {
  return axes[id].searchSteps;
}

// ==================== MOVEMENT ====================
MoveStart motorsMoveTo(AxisId id, long target) {
  Axis &a = axes[id];
  if (!a.referenced) return MOVE_NOT_REFERENCED;

  target = constrain(target, 0L, a.maxTravel);
  a.target = a.sign * target;

  if (a.target == a.pos) {
    clearMotion(a);
    return MOVE_ALREADY_THERE;
  }

  a.moving = true;
  a.stepDir = (a.target > a.pos) ? 1 : -1;
  a.intervalUs = (unsigned long)(1000000.0f / a.maxSpeed);
  a.lastStepUs = micros();
  return MOVE_STARTED;
}

MoveStatus motorsRunAxis(AxisId id) {
  Axis &a = axes[id];

  // Remaining logical distance < 0 means the axis is moving toward zero.
  if (a.stopOnLimit && a.moving && (a.sign * (a.target - a.pos)) < 0 &&
      sensorsLimitActive(a.limitPin)) {
    clearMotion(a);
    a.pos = 0;
    a.target = 0;
    return MOVE_LIMIT_HIT;
  }

  return stepAxisMove(a, micros()) ? MOVE_RUNNING : MOVE_DONE;
}

bool motorsRunAll() {
  bool running = false;
  for (uint8_t i = 0; i < AXIS_COUNT; i++) {
    if (axes[i].moving && motorsRunAxis((AxisId)i) == MOVE_RUNNING) {
      running = true;
    }
  }
  return running;
}

// ==================== QUERIES ====================
long motorsPosition(AxisId id) {
  return axes[id].sign * axes[id].pos;
}

long motorsTarget(AxisId id) {
  return axes[id].sign * axes[id].target;
}

bool motorsIsReferenced(AxisId id) {
  return axes[id].referenced;
}

bool motorsAllReferenced() {
  for (uint8_t i = 0; i < AXIS_COUNT; i++) {
    if (!axes[i].referenced) return false;
  }
  return true;
}

// ==================== STOP ====================
void motorsStopAll() {
  for (uint8_t i = 0; i < AXIS_COUNT; i++) {
    clearMotion(axes[i]);
    axes[i].target = axes[i].pos;
    axes[i].referenced = false;
  }
}