#include "motors.h"
#include "sensors.h"
#include "servo.h"

// ==================== CONFIG ====================
// Start position (user steps)
const long START_X = 8000L;
const long START_Y = 5500L;
const long START_A = 1200L;
const long START_Z = 0L;

const unsigned long DELAY_BEFORE_A_MS          = 800;
const unsigned long DELAY_BETWEEN_WAYPOINTS_MS = 2000;
const unsigned long LIVE_LOG_INTERVAL_MS       = 250;
const unsigned long COMMAND_TIMEOUT_MS         = 120;

// Homing order
// Axes that search their limit switch (Z is NOT homed)
const AxisId HOMING_AXES[] = { AXIS_X, AXIS_Y, AXIS_A };
const byte HOMING_COUNT = sizeof(HOMING_AXES) / sizeof(HOMING_AXES[0]);

// Run the route automatically after the boot homing
const bool AUTO_START_ROUTE = true;
// Route. servoAngle < 0 means no servo action on arrival.
const int NO_SERVO = -1;

struct Waypoint {
  long x, y, a, z;
  int servoAngle;
};

const Waypoint ROUTE[] = {
  {  8000L, 5500L, 1200L, 41000L, SERVO_CLOSED_ANGLE },  // 1
  {  8000L, 5500L, 1200L,     0L, NO_SERVO },            // 1.1
  { 16000L, 5500L, 1200L,     0L, NO_SERVO },            // 1.2
  { 16000L, 5500L, 1200L, 41000L, SERVO_OPEN_ANGLE },    // 1.3
  { 16000L, 5500L, 1200L,     0L, NO_SERVO },            // 1.4
};
const byte ROUTE_LENGTH = sizeof(ROUTE) / sizeof(ROUTE[0]);

// ==================== SEQUENCER STATE ====================
enum Mode : uint8_t {
  MODE_IDLE,
  MODE_RAISE_Z,
  MODE_HOMING,
  MODE_GO_START_XYZ,
  MODE_START_PAUSE,
  MODE_GO_START_A,
  MODE_ROUTE_MOVE,
  MODE_ROUTE_WAIT,
  MODE_ERROR
};

Mode mode = MODE_IDLE;
bool runAfterHoming = false;
byte routeIndex = 0;
unsigned long timerStart = 0;

bool liveLog = true;
unsigned long lastLiveLog = 0;

// ==================== HELPERS ====================
static const __FlashStringHelper* modeName() {
  switch (mode) {
    case MODE_IDLE:          return F("IDLE");
    case MODE_HOMING:        return F("HOMING");
    case MODE_GO_START_XYZ:  return F("GO_START_XYZ");
    case MODE_START_PAUSE:   return F("START_PAUSE");
    case MODE_GO_START_A:    return F("GO_START_A");
    case MODE_ROUTE_MOVE:    return F("ROUTE_MOVE");
    case MODE_ROUTE_WAIT:    return F("ROUTE_WAIT");
    case MODE_ERROR:         return F("ERROR");
    case MODE_RAISE_Z:       return F("RAISE_Z");
  }
  return F("?");
}

static bool busy() {
  return (mode != MODE_IDLE && mode != MODE_ERROR) || !axesIdle();
}

static void enterError(const __FlashStringHelper* reason) {
  motorsStop();
  mode = MODE_ERROR;
  Serial.print(F("[SEQ] ERROR: "));
  Serial.println(reason);
  Serial.println(F("[SEQ] Send H to re-home or P to clear."));
}

static void printStatus() {
  Serial.print(F("[STATUS] mode="));
  Serial.println(modeName());
  Serial.print(F("[STATUS] pos: "));
  motorsPrintPositions();
  motorsPrintSettings();
  sensorsPrintStatus();
}

static void printHelp() {
  Serial.println(F("[HELP] X Y A Z     home one axis"));
  Serial.println(F("[HELP] X500        move axis to position (steps)"));
  Serial.println(F("[HELP] H           home all axes"));
  Serial.println(F("[HELP] R / START   run route"));
  Serial.println(F("[HELP] P / STOP    stop (re-homing required)"));
  Serial.println(F("[HELP] S4000       set speed (steps/s), S alone = show"));
  Serial.println(F("[HELP] C2000       set acceleration, C alone = show"));
  Serial.println(F("[HELP] L           toggle live position log"));
  Serial.println(F("[HELP] ? / STATUS  show status"));
}

// ==================== SEQUENCES ====================
static void startHomingSequence(bool thenRunRoute) {
  runAfterHoming = thenRunRoute;
  Serial.println(F("[SEQ] Z has no limit search. Raising Z to 0, then homing X, Y, A together"));
  if (!moveAxis(AXIS_Z, 0)) { enterError(F("Z move rejected")); return; }
  mode = MODE_RAISE_Z;
}

static void goToStart() {
  bool ok = moveAxis(AXIS_X, START_X);
  ok &= moveAxis(AXIS_Y, START_Y);
  ok &= moveAxis(AXIS_Z, START_Z);
  if (!ok) { enterError(F("start move rejected")); return; }
  mode = MODE_GO_START_XYZ;
}

static void startWaypoint(byte index) {
  const Waypoint& w = ROUTE[index];
  Serial.print(F("[SEQ] Waypoint "));
  Serial.print(index + 1);
  Serial.print('/');
  Serial.println(ROUTE_LENGTH);

  bool ok = moveAxis(AXIS_X, w.x);
  ok &= moveAxis(AXIS_Y, w.y);
  ok &= moveAxis(AXIS_A, w.a);
  ok &= moveAxis(AXIS_Z, w.z);
  if (!ok) { enterError(F("waypoint move rejected")); return; }
  mode = MODE_ROUTE_MOVE;
}

static void startRoute() {
  routeIndex = 0;
  Serial.println(F("[SEQ] Route started"));
  startWaypoint(0);
}

static void stopAll() {
  motorsStop();
  mode = MODE_IDLE;
  Serial.println(F("[SEQ] Stopped. Send H to re-home."));
}

static void updateSequence() {
  if (mode != MODE_ERROR && anyAxisError()) {
    enterError(F("limit switch not found"));
    return;
  }

  switch (mode) {
    

    case MODE_HOMING:
      if (axesIdle()) {
        Serial.println(F("[SEQ] X, Y, A homed. Going to start position..."));
        goToStart();
      }
      break;

    case MODE_GO_START_XYZ:
      if (axesIdle()) {
        timerStart = millis();
        mode = MODE_START_PAUSE;
        Serial.println(F("[SEQ] X, Y, Z in place. Pause before moving A..."));
      }
      break;

    case MODE_START_PAUSE:
      if (millis() - timerStart >= DELAY_BEFORE_A_MS) {
        if (!moveAxis(AXIS_A, START_A)) { enterError(F("A move rejected")); break; }
        mode = MODE_GO_START_A;
      }
      break;
      
    case MODE_GO_START_A:
      if (axesIdle()) {
        Serial.print(F("[SEQ] Start position reached: "));
        motorsPrintPositions();
        if (runAfterHoming) {
          runAfterHoming = false;
          Serial.println(F("[SEQ] Auto-starting route..."));
          startRoute();
        } else {
          mode = MODE_IDLE;
          Serial.println(F("[SEQ] Send R or START to run the route."));
        }
      }
      break;

    case MODE_ROUTE_MOVE:
      if (axesIdle()) {
        int angle = ROUTE[routeIndex].servoAngle;
        if (angle != NO_SERVO) servoMove(angle);
        timerStart = millis();
        mode = MODE_ROUTE_WAIT;
        Serial.print(F("[SEQ] Waypoint reached: "));
        motorsPrintPositions();
      }
      break;

    case MODE_ROUTE_WAIT:
      if (millis() - timerStart >= DELAY_BETWEEN_WAYPOINTS_MS) {
        routeIndex++;
        if (routeIndex >= ROUTE_LENGTH) {
          Serial.println(F("[SEQ] Route finished. Returning to start position..."));
          goToStart();
        } else {
          startWaypoint(routeIndex);
        }
      }
      break;
    
    case MODE_RAISE_Z:
      if (axesIdle()) {
        for (byte i = 0; i < HOMING_COUNT; i++) homeAxis(HOMING_AXES[i]);
        mode = MODE_HOMING;
      }
      break;

    default:
      break;
  }
}

// ==================== COMMANDS ====================
static bool onlyDigits(const char* s) {
  if (*s == '\0') return false;
  while (*s) { if (*s < '0' || *s > '9') return false; s++; }
  return true;
}

static bool axisFromChar(char c, AxisId& id) {
  for (byte i = 0; i < AXIS_COUNT; i++) {
    if (axisName((AxisId)i) == c) { id = (AxisId)i; return true; }
  }
  return false;
}

static void processCommand(const char* raw) {
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

  Serial.print(F("[CMD] "));
  Serial.println(cmd);

  // ---- always allowed ----
  if (!strcmp(cmd, "P") || !strcmp(cmd, "STOP")) { stopAll(); return; }
  if (!strcmp(cmd, "?") || !strcmp(cmd, "POS") || !strcmp(cmd, "STATUS")) { printStatus(); return; }
  if (!strcmp(cmd, "HELP")) { printHelp(); return; }
  if (!strcmp(cmd, "L")) {
    liveLog = !liveLog;
    Serial.println(liveLog ? F("[CMD] Live log ON") : F("[CMD] Live log OFF"));
    return;
  }

  // ---- run / home all ----
  if (!strcmp(cmd, "R") || !strcmp(cmd, "START")) {
    if (busy() || !allHomed()) {
      Serial.println(F("[CMD] Cannot start: wait for homing to finish (or send H)."));
      return;
    }
    startRoute();
    return;
  }
  if (!strcmp(cmd, "H") || !strcmp(cmd, "HOME")) {
    if (busy()) { Serial.println(F("[CMD] Busy. Wait or send P.")); return; }
    startHomingSequence(false);
    return;
  }

  // ---- speed / acceleration (also allowed while moving) ----
  if (cmd[0] == 'S' || cmd[0] == 'C') {
    if (cmd[1] == '\0') { motorsPrintSettings(); return; }
    if (!onlyDigits(cmd + 1)) { Serial.println(F("[CMD] Invalid number.")); return; }
    float value = (float)atol(cmd + 1);
    if (cmd[0] == 'S') setTravelSpeed(value); else setAcceleration(value);
    motorsPrintSettings();
    return;
  }

  // ---- single axis ----
  if (busy()) { Serial.println(F("[CMD] Busy. Wait or send P.")); return; }

  AxisId id;
  if (!axisFromChar(cmd[0], id)) { Serial.println(F("[CMD] Unknown command. Send HELP.")); return; }

  if (cmd[1] == '\0') { homeAxis(id); return; }
  if (!onlyDigits(cmd + 1)) { Serial.println(F("[CMD] Invalid format.")); return; }
  moveAxis(id, atol(cmd + 1));
}

// ==================== SERIAL READER ====================
char serialBuffer[20];
byte serialIndex = 0;
unsigned long lastCharTime = 0;

static void readSerial() {
  while (Serial.available() > 0) {
    char ch = (char)Serial.read();
    lastCharTime = millis();
    if (ch == '\r') continue;
    if (ch == '\n') {
      serialBuffer[serialIndex] = '\0';
      processCommand(serialBuffer);
      serialIndex = 0;
    } else if (ch >= 32 && ch <= 126 && serialIndex < sizeof(serialBuffer) - 1) {
      serialBuffer[serialIndex++] = ch;
    }
  }
  // Terminal without newline: process after a short idle time
  if (serialIndex > 0 && millis() - lastCharTime >= COMMAND_TIMEOUT_MS) {
    serialBuffer[serialIndex] = '\0';
    processCommand(serialBuffer);
    serialIndex = 0;
  }
}

static void updateLiveLog() {
  if (!liveLog || axesIdle()) return;
  unsigned long now = millis();
  if (now - lastLiveLog < LIVE_LOG_INTERVAL_MS) return;
  lastLiveLog = now;
  Serial.print(F("[LIVE] "));
  Serial.print(now);
  Serial.print(F(" ms | "));
  motorsPrintPositions();
}

// ==================== SETUP / LOOP ====================
void setup() {
  Serial.begin(115200);
  Serial.println(F("[SYS] SCARA X Y A Z controller"));
  printHelp();

  servoInit();
  sensorsInit();
  motorsInit();
  sensorsPrintStatus();

  setZeroHere(AXIS_Z);                    // Z assumed at its top position (0)
  startHomingSequence(AUTO_START_ROUTE);
}

void loop() {
  readSerial();
  sensorsUpdate();
  motorsUpdate();
  updateSequence();
  updateLiveLog();
}