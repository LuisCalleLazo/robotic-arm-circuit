#include "motors.h"
#include "sensors.h"
#include "servo.h"
#include "z_axis.h"

// ==================== SERIAL PROTOCOL (FOR THE HMI) ====================
// Every line sent by the Arduino is CSV: TAG,field,field,...
//
//   STATE,<name>                     state machine changed state
//   EVT,<name>[,<axis>,<value>]      event (homing, moves, route, servo)
//   POS,X=<n>,Y=<n>,A=<n>,Z=<n>      current logical positions in steps
//   ERR,<code>[,<axis>]              error
//   CMD,<text>                       echo of the command received
//   INFO,<text>                      free text
//
// Commands accepted (newline terminated, case and spaces ignored):
//   X | Y | A | Z     home that axis
//   X500 | Y200 ...   move that axis to the given step position
//   R | START | INICIAR   run the route (all axes must be homed)
//   P | STOP              stop everything (axes must be homed again)
//   ? | POS | POSICION    print state and positions

// ==================== TIMING CONFIGURATION ====================
// Pause after X, Y and Z reach the home pose and before A moves.
const unsigned long DELAY_BEFORE_A_MS = 800;
// Pause after each route pose is reached.
const unsigned long DELAY_BETWEEN_POSES_MS = 2000;
// Position streaming period while moving. Set to 0 to disable.
// Keep it above ~150 ms so serial output does not disturb step timing.
const unsigned long POS_STREAM_INTERVAL_MS = 250;

// ==================== POSES AND ROUTE ====================
const int SERVO_NONE = -1;

struct Pose {
  long x, y, a, z;
  int servoAngle;  // moved when the pose is reached, SERVO_NONE to skip
};

// Pose reached after homing and after the route ends.
const Pose HOME_POSE = { 8000, 5500, 1200, 0, SERVO_NONE };

// Route executed by R / START. Edit this table to change the sequence.
const Pose ROUTE[] = {
  //  X      Y     A     Z      servo on arrival
  {  8000, 5500, 1200, 41000, SERVO_CLOSED_ANGLE },  // 1   : pick
  {  8000, 5500, 1200,     0, SERVO_NONE },          // 1.1 : lift
  { 16000, 5500, 1200,     0, SERVO_NONE },          // 1.2 : travel
  { 16000, 5500, 1200, 41000, SERVO_OPEN_ANGLE },    // 1.3 : place
  { 16000, 5500, 1200,     0, SERVO_NONE }           // 1.4 : lift
};
const uint8_t ROUTE_LENGTH = sizeof(ROUTE) / sizeof(ROUTE[0]);

// Homing order at power on.
const AxisId HOMING_ORDER[AXIS_COUNT] = { AXIS_Z, AXIS_X, AXIS_A, AXIS_Y };

// ==================== STATE MACHINE ====================
enum State : uint8_t {
  ST_IDLE,
  ST_HOMING,
  ST_MOVING_AXIS,
  ST_GOTO_HOME_POSE,
  ST_WAIT_BEFORE_A,
  ST_MOVING_A,
  ST_ROUTE_MOVING,
  ST_ROUTE_WAIT,
  ST_ERROR
};

State state = ST_IDLE;
AxisId activeAxis = AXIS_X;
bool autoHoming = false;
uint8_t homingIndex = 0;
uint8_t routeStep = 0;
unsigned long waitStart = 0;
unsigned long lastPosLog = 0;

// ==================== LOGGING ====================
const __FlashStringHelper *stateName(State s) {
  switch (s) {
    case ST_IDLE:           return F("IDLE");
    case ST_HOMING:         return F("HOMING");
    case ST_MOVING_AXIS:    return F("MOVING_AXIS");
    case ST_GOTO_HOME_POSE: return F("GOTO_HOME_POSE");
    case ST_WAIT_BEFORE_A:  return F("WAIT_BEFORE_A");
    case ST_MOVING_A:       return F("MOVING_A");
    case ST_ROUTE_MOVING:   return F("ROUTE_MOVING");
    case ST_ROUTE_WAIT:     return F("ROUTE_WAIT");
    default:                return F("ERROR");
  }
}

void setState(State s) {
  if (s == state) return;
  state = s;
  Serial.print(F("STATE,"));
  Serial.println(stateName(s));
}

void logState() {
  Serial.print(F("STATE,"));
  Serial.println(stateName(state));
}

void logPosition() {
  Serial.print(F("POS"));
  for (uint8_t i = 0; i < AXIS_COUNT; i++) {
    AxisId id = (AxisId)i;
    Serial.print(',');
    Serial.print(motorsName(id));
    Serial.print('=');
    Serial.print(id == AXIS_Z ? zAxisPosition() : motorsPosition(id));
  }
  Serial.println();
}

void logEvent(const __FlashStringHelper *name) {
  Serial.print(F("EVT,"));
  Serial.println(name);
}

void logAxisEvent(const __FlashStringHelper *name, AxisId id, long value) {
  Serial.print(F("EVT,"));
  Serial.print(name);
  Serial.print(',');
  Serial.print(motorsName(id));
  Serial.print(',');
  Serial.println(value);
}

void logError(const __FlashStringHelper *code) {
  Serial.print(F("ERR,"));
  Serial.println(code);
}

void logAxisError(const __FlashStringHelper *code, AxisId id) {
  Serial.print(F("ERR,"));
  Serial.print(code);
  Serial.print(',');
  Serial.println(motorsName(id));
}

bool isMovingState() {
  return state == ST_MOVING_AXIS || state == ST_GOTO_HOME_POSE ||
         state == ST_MOVING_A || state == ST_ROUTE_MOVING;
}

// Streams positions while an axis is moving so the HMI can animate.
void streamPosition() {
  if (POS_STREAM_INTERVAL_MS == 0 || !isMovingState()) return;
  if (millis() - lastPosLog >= POS_STREAM_INTERVAL_MS) {
    lastPosLog = millis();
    logPosition();
  }
}

// ==================== ACTIONS ====================
// Starts the homing search of one axis.
// Returns true if a search started, false if the limit was already active.
bool beginHoming(AxisId id) {
  bool started = id == AXIS_Z ? zAxisBeginHoming() : motorsHomingStart(id);
  if (started) {
    logAxisEvent(F("HOMING_START"), id, 0);
    setState(ST_HOMING);
    return true;
  }
  logAxisEvent(F("HOMING_DONE"), id, 0);
  return false;
}

// Sets the X, Y, A and Z targets of a pose. Start of a simultaneous move.
void setPoseTargets(const Pose &p) {
  motorsMoveTo(AXIS_X, p.x);
  motorsMoveTo(AXIS_Y, p.y);
  motorsMoveTo(AXIS_A, p.a);
  zAxisMoveTo(p.z);
}

bool runAllAxes() {
  bool otherAxesRunning = motorsRunAll();
  MoveStatus zStatus = zAxisRun();
  return otherAxesRunning || zStatus == MOVE_RUNNING;
}

// Moves X, Y and Z to the home pose first; A moves later (see ST_WAIT_BEFORE_A).
void goToHomePose() {
  motorsMoveTo(AXIS_X, HOME_POSE.x);
  motorsMoveTo(AXIS_Y, HOME_POSE.y);
  zAxisMoveTo(HOME_POSE.z);
  logEvent(F("HOME_POSE_START"));
  setState(ST_GOTO_HOME_POSE);
}

// Advances the automatic homing sequence to the next axis that needs a search.
// When every axis is referenced it moves to the home pose.
void continueAutoHoming() {
  while (homingIndex < AXIS_COUNT) {
    activeAxis = HOMING_ORDER[homingIndex];
    if (beginHoming(activeAxis)) return;
    homingIndex++;
  }
  autoHoming = false;
  logEvent(F("HOMING_COMPLETE"));
  goToHomePose();
}

void startAutoHoming() {
  autoHoming = true;
  homingIndex = 0;
  continueAutoHoming();
}

void startRouteStep() {
  if (routeStep >= ROUTE_LENGTH) {
    logEvent(F("ROUTE_DONE"));
    goToHomePose();
    return;
  }
  setPoseTargets(ROUTE[routeStep]);
  Serial.print(F("EVT,ROUTE_STEP,"));
  Serial.println(routeStep);
  setState(ST_ROUTE_MOVING);
}

void startRoute() {
  if (state != ST_IDLE || !motorsAllReferenced() || !zAxisIsReferenced()) {
    logError(F("NOT_READY"));
    return;
  }
  routeStep = 0;
  logEvent(F("ROUTE_START"));
  startRouteStep();
}

void stopAll() {
  motorsStopAll();
  zAxisStop();
  autoHoming = false;
  logEvent(F("STOPPED"));
  setState(ST_IDLE);
}

// Runs the servo action of a pose and reports the result.
void applyServo(int angle) {
  if (angle == SERVO_NONE) return;
  if (servoMove(angle)) {
    Serial.print(F("EVT,SERVO,"));
    Serial.println(angle);
  } else {
    logError(F("SERVO_FAILED"));
  }
}

// ==================== COMMAND PARSER ====================
bool onlyDigits(const char *s) {
  if (*s == '\0') return false;
  while (*s) {
    if (*s < '0' || *s > '9') return false;
    s++;
  }
  return true;
}

void processCommand(char *raw) {
  // Remove spaces, tabs and '=', convert to upper case.
  char cmd[20];
  byte n = 0;
  for (byte i = 0; raw[i] && n < sizeof(cmd) - 1; i++) {
    char ch = raw[i];
    if (ch == ' ' || ch == '\t' || ch == '=') continue;
    if (ch >= 'a' && ch <= 'z') ch -= 32;
    cmd[n++] = ch;
  }
  cmd[n] = '\0';
  if (!n) return;

  Serial.print(F("CMD,"));
  Serial.println(cmd);

  if (!strcmp(cmd, "?") || !strcmp(cmd, "POS") || !strcmp(cmd, "POSICION")) {
    logState();
    logPosition();
    return;
  }

  // Same as the original: after a homing error nothing else is accepted
  // until the board is reset.
  if (state == ST_ERROR) { logError(F("RESET_REQUIRED")); return; }

  if (!strcmp(cmd, "P") || !strcmp(cmd, "STOP")) { stopAll(); return; }
  if (!strcmp(cmd, "R") || !strcmp(cmd, "START") || !strcmp(cmd, "INICIAR")) {
    startRoute();
    return;
  }

  if (state != ST_IDLE) { logError(F("BUSY")); return; }

  AxisId id;
  if (!motorsAxisFromChar(cmd[0], &id)) { logError(F("UNKNOWN_COMMAND")); return; }

  // Single letter: home that axis.
  if (cmd[1] == '\0') {
    autoHoming = false;
    activeAxis = id;
    beginHoming(id);
    return;
  }

  // Letter plus number: move that axis.
  if (!onlyDigits(cmd + 1)) { logError(F("BAD_FORMAT")); return; }

  MoveStart moveResult = id == AXIS_Z
    ? zAxisMoveTo(atol(cmd + 1))
    : motorsMoveTo(id, atol(cmd + 1));
  switch (moveResult) {
    case MOVE_NOT_REFERENCED:
      logAxisError(F("NOT_REFERENCED"), id);
      break;
    case MOVE_ALREADY_THERE:
      logAxisEvent(F("MOVE_DONE"), id, id == AXIS_Z ? zAxisPosition() : motorsPosition(id));
      break;
    case MOVE_STARTED:
      activeAxis = id;
      logAxisEvent(F("MOVE_START"), id, id == AXIS_Z ? zAxisTarget() : motorsTarget(id));
      setState(ST_MOVING_AXIS);
      break;
  }
}

// ==================== SERIAL READER ====================
// A command ends with a newline, or after TIME_END_COMMAND_MS without new
// characters (useful when the terminal does not send a line ending).
char serialBuffer[20];
byte serialIndex = 0;
unsigned long lastSerialChar = 0;
const unsigned long TIME_END_COMMAND_MS = 120;

void readSerial() {
  while (Serial.available() > 0) {
    char ch = (char)Serial.read();
    lastSerialChar = millis();
    if (ch == '\r') continue;
    if (ch == '\n') {
      serialBuffer[serialIndex] = '\0';
      processCommand(serialBuffer);
      serialIndex = 0;
    } else if (ch >= 32 && ch <= 126 && serialIndex < sizeof(serialBuffer) - 1) {
      serialBuffer[serialIndex++] = ch;
    }
  }
  if (serialIndex > 0 && millis() - lastSerialChar >= TIME_END_COMMAND_MS) {
    serialBuffer[serialIndex] = '\0';
    processCommand(serialBuffer);
    serialIndex = 0;
  }
}

// ==================== STATE MACHINE UPDATE ====================
void updateStateMachine() {
  switch (state) {

    case ST_HOMING: {
      HomingStatus status = activeAxis == AXIS_Z
        ? zAxisUpdateHoming()
        : motorsHomingUpdate(activeAxis);
      if (status == HOMING_DONE) {
        long searched = activeAxis == AXIS_Z
          ? zAxisSearchSteps()
          : motorsSearchSteps(activeAxis);
        logAxisEvent(F("HOMING_DONE"), activeAxis, searched);
        if (autoHoming) {
          homingIndex++;
          continueAutoHoming();
        } else {
          setState(ST_IDLE);
        }
      } else if (status == HOMING_FAILED) {
        logAxisError(F("HOMING_LIMIT_NOT_FOUND"), activeAxis);
        autoHoming = false;
        setState(ST_ERROR);
      }
      break;
    }

    case ST_MOVING_AXIS: {
      MoveStatus status = activeAxis == AXIS_Z
        ? zAxisRun()
        : motorsRunAxis(activeAxis);
      if (status == MOVE_DONE) {
        long position = activeAxis == AXIS_Z
          ? zAxisPosition()
          : motorsPosition(activeAxis);
        logAxisEvent(F("MOVE_DONE"), activeAxis, position);
        setState(ST_IDLE);
      } else if (status == MOVE_LIMIT_HIT) {
        logAxisEvent(F("LIMIT_HIT"), activeAxis, 0);
        setState(ST_IDLE);
      }
      break;
    }

    case ST_GOTO_HOME_POSE:
      if (!runAllAxes()) {
        waitStart = millis();
        setState(ST_WAIT_BEFORE_A);
      }
      break;

    case ST_WAIT_BEFORE_A:
      if (millis() - waitStart >= DELAY_BEFORE_A_MS) {
        motorsMoveTo(AXIS_A, HOME_POSE.a);
        setState(ST_MOVING_A);
      }
      break;

    case ST_MOVING_A:
      if (motorsRunAxis(AXIS_A) != MOVE_RUNNING) {
        logEvent(F("HOME_POSE_REACHED"));
        logPosition();
        setState(ST_IDLE);
      }
      break;

    case ST_ROUTE_MOVING:
      if (!runAllAxes()) {
        applyServo(ROUTE[routeStep].servoAngle);
        logPosition();
        waitStart = millis();
        setState(ST_ROUTE_WAIT);
      }
      break;

    case ST_ROUTE_WAIT:
      if (millis() - waitStart >= DELAY_BETWEEN_POSES_MS) {
        routeStep++;
        startRouteStep();
      }
      break;

    case ST_ERROR:
      // Nothing moves. Reset the board and check the limit switch.
      break;

    default:
      break;
  }
}

// ==================== SETUP / LOOP ====================
void setup() {
  Serial.begin(115200);
  Serial.setTimeout(100);

  logEvent(F("BOOT"));

  sensorsInit();
  motorsInit();
  zAxisInit();

  if (servoInit()) {
    logEvent(F("SERVO_READY"));
    servoMove(SERVO_OPEN_ANGLE);
  } else {
    logError(F("SERVO_NOT_FOUND"));
  }

  Serial.println(F("INFO,SCARA X Y A Z"));
  Serial.println(F("INFO,Commands: X Y A Z | X500 | R | P | POS"));

  startAutoHoming();
}

void loop() {
  readSerial();
  updateStateMachine();
  streamPosition();
}
