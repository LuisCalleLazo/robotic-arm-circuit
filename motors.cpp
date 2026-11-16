#include "motors.h"
#include "sensors.h"

// X, Y and A use the custom ramp. Z has its own controller in z_axis.cpp.
//
// sign: converts logical position to motor steps (motorSteps = sign * logical).
//       -1 inverts the physical direction. Homing always moves toward the
//       limit switch, i.e. toward logical negative.
// stopOnLimit: if true, a normal move toward zero is cancelled when the
//       limit switch activates (used by Z as a crash protection).
// limitedSearch: if true, homing fails after HOMING_MAX_STEPS (X, Y, A).
struct Axis {
  // ----- fixed configuration -----
  uint8_t stepPin, dirPin, limitPin;
  char name;
  int8_t sign;
  long maxTravel;
  float maxSpeed;      // steps/s
  float accel;         // steps/s^2
  bool stopOnLimit;
  bool limitedSearch;

  // ----- runtime state (motor domain: motor steps, not logical) -----
  bool referenced;
  bool moving;         // true while a ramped move is in progress
  long searchSteps;
  long pos;            // current position in motor steps
  long target;         // target position in motor steps
  float speed;         // current speed magnitude, steps/s
  unsigned long intervalUs;   // time between steps, 0 = not stepping
  unsigned long lastStepUs;   // time of the last step
  int8_t stepDir;      // direction of the next steps: +1 or -1
  int8_t dirLevel;     // level currently on the DIR pin: +1, -1, 0 = unknown

  Axis(uint8_t sp, uint8_t dp, uint8_t lp, char n, int8_t s, long travel,
       float vmax, float acc, bool stopLimit, bool limitSearch)
    : stepPin(sp), dirPin(dp), limitPin(lp), name(n), sign(s),
      maxTravel(travel), maxSpeed(vmax), accel(acc),
      stopOnLimit(stopLimit), limitedSearch(limitSearch),
      referenced(false), moving(false), searchSteps(0), pos(0), target(0),
      speed(0), intervalUs(0), lastStepUs(0), stepDir(1), dirLevel(0) {}
};

//                 step pin    dir pin    limit pin    name sign travel        speed        accel        stopOnLimit limitedSearch
static Axis axes[AXIS_COUNT] = {
  Axis(STEP_X_PIN, DIR_X_PIN, LIMIT_X_PIN, 'X', -1, MAX_TRAVEL_X, MOVE_SPEED_X, MOVE_ACCEL_X, false, true),
  Axis(STEP_Y_PIN, DIR_Y_PIN, LIMIT_Y_PIN, 'Y', -1, MAX_TRAVEL_Y, MOVE_SPEED_Y, MOVE_ACCEL_Y, false, true),
  Axis(STEP_A_PIN, DIR_A_PIN, LIMIT_A_PIN, 'A',  1, MAX_TRAVEL_A, MOVE_SPEED_A, MOVE_ACCEL_A, false, true),
  Axis(STEP_Z_PIN, DIR_Z_PIN, LIMIT_Z_PIN, 'Z',  1, MAX_TRAVEL_Z, MOVE_SPEED_Z, 0, true, false)
};

// Time of the last ramp update (shared by all axes).
static unsigned long lastRampUs = 0;
// A ramp update never integrates more than this, so the first update after
// an idle period does not make the speed jump.
static const unsigned long RAMP_MAX_ELAPSED_US = 10000;

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
static bool stepIfDue(Axis &a) {
  if (a.intervalUs == 0) return false;
  unsigned long now = micros();
  if (now - a.lastStepUs < a.intervalUs) return false;
  a.lastStepUs += a.intervalUs;
  if (now - a.lastStepUs >= a.intervalUs) a.lastStepUs = now;
  doStep(a);
  return true;
}

static void clearMotion(Axis &a) {
  a.moving = false;
  a.speed = 0;
  a.intervalUs = 0;
}

// ==================== SPEED RAMP ====================
// Trapezoidal profile without square roots: if the speed is large enough
// that the axis could not stop before the target at the configured
// acceleration (v^2 >= 2*a*distance), decelerate; otherwise accelerate up to
// the maximum speed.
static void rampUpdate(Axis &a, float dt) {
  long remaining = a.target - a.pos;
  if (remaining == 0) { clearMotion(a); return; }

  long dist = (remaining > 0) ? remaining : -remaining;
  a.stepDir = (remaining > 0) ? 1 : -1;

  float v = a.speed;
  if (v * v >= 2.0f * a.accel * (float)dist) v -= a.accel * dt;
  else                                        v += a.accel * dt;

  if (v > a.maxSpeed) v = a.maxSpeed;
  if (v < RAMP_START_SPEED) v = RAMP_START_SPEED;

  a.speed = v;
  a.intervalUs = (unsigned long)(1000000.0f / v);
}

// Updates the custom ramp of X, Y and A every RAMP_UPDATE_US.
static void rampTick() {
  unsigned long now = micros();
  unsigned long elapsed = now - lastRampUs;
  if (elapsed < RAMP_UPDATE_US) return;
  lastRampUs = now;
  if (elapsed > RAMP_MAX_ELAPSED_US) elapsed = RAMP_MAX_ELAPSED_US;
  float dt = (float)elapsed * 1.0e-6f;

  for (uint8_t i = 0; i < AXIS_COUNT; i++) {
    if (axes[i].moving) rampUpdate(axes[i], dt);
  }
}

// Sends the due step of one moving axis. Returns true while it is moving.
static bool stepAxisMove(Axis &a) {
  if (!a.moving) return false;

  long remaining = a.target - a.pos;
  if (remaining == 0) { clearMotion(a); return false; }

  // Direction mismatch (should not happen): wait for the next ramp update.
  if ((remaining > 0) != (a.stepDir > 0)) return true;

  stepIfDue(a);
  return true;
}

// ==================== INIT AND LOOKUP ====================
void motorsInit() {
  for (uint8_t i = 0; i < AXIS_Z; i++) {
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
  if (id == AXIS_Z) return false;
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

  // Constant-speed search: speed is set by HOMING_SPEED in motors.h.
  a.stepDir = -a.sign;
  a.intervalUs = (unsigned long)(1000000.0f / HOMING_SPEED);
  a.lastStepUs = micros();
  return true;
}

HomingStatus motorsHomingUpdate(AxisId id) {
  if (id == AXIS_Z) return HOMING_FAILED;
  Axis &a = axes[id];

  if (sensorsLimitActive(a.limitPin)) {
    clearMotion(a);
    a.pos = 0;
    a.target = 0;
    a.referenced = true;
    return HOMING_DONE;
  }
  if (a.limitedSearch && a.searchSteps >= HOMING_MAX_STEPS) {
    return HOMING_FAILED;
  }
  if (stepIfDue(a)) {
    a.searchSteps++;
  }
  return HOMING_RUNNING;
}

long motorsSearchSteps(AxisId id) {
  return axes[id].searchSteps;
}

// ==================== MOVEMENT ====================
MoveStart motorsMoveTo(AxisId id, long target) {
  if (id == AXIS_Z) return MOVE_NOT_REFERENCED;
  Axis &a = axes[id];
  if (!a.referenced) return MOVE_NOT_REFERENCED;

  target = constrain(target, 0L, a.maxTravel);
  a.target = a.sign * target;

  if (a.target == a.pos) {
    clearMotion(a);
    return MOVE_ALREADY_THERE;
  }

  // The ramp starts from zero speed; the next ramp update sets the speed.
  a.moving = true;
  a.speed = 0;
  a.intervalUs = 0;
  a.stepDir = (a.target > a.pos) ? 1 : -1;
  a.lastStepUs = micros();
  return MOVE_STARTED;
}

MoveStatus motorsRunAxis(AxisId id) {
  if (id == AXIS_Z) return MOVE_DONE;
  Axis &a = axes[id];

  // Remaining logical distance < 0 means the axis is moving toward zero.
  if (a.stopOnLimit && a.moving && (a.sign * (a.target - a.pos)) < 0 &&
      sensorsLimitActive(a.limitPin)) {
    clearMotion(a);
    a.pos = 0;
    a.target = 0;
    return MOVE_LIMIT_HIT;
  }

  rampTick();
  return stepAxisMove(a) ? MOVE_RUNNING : MOVE_DONE;
}

bool motorsRunAll() {
  rampTick();
  bool running = false;
  for (uint8_t i = 0; i < AXIS_Z; i++) {
    if (stepAxisMove(axes[i])) running = true;
  }
  return running;
}

// ==================== QUERIES ====================
long motorsPosition(AxisId id) {
  if (id == AXIS_Z) return 0;
  return axes[id].sign * axes[id].pos;
}

long motorsTarget(AxisId id) {
  if (id == AXIS_Z) return 0;
  return axes[id].sign * axes[id].target;
}

bool motorsIsReferenced(AxisId id) {
  if (id == AXIS_Z) return false;
  return axes[id].referenced;
}

bool motorsAllReferenced() {
  for (uint8_t i = 0; i < AXIS_Z; i++) {
    if (!axes[i].referenced) return false;
  }
  return true;
}

// ==================== STOP ====================
void motorsStopAll() {
  for (uint8_t i = 0; i < AXIS_Z; i++) {
    clearMotion(axes[i]);
    axes[i].target = axes[i].pos;
    axes[i].referenced = false;
  }
}