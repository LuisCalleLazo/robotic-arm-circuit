#include "sensors.h"

static const uint8_t PINS[LIMIT_COUNT] = { 11, 10, 9, 8 };
static const char NAMES[LIMIT_COUNT]   = { 'X', 'Y', 'A', 'Z' };
static const unsigned long POLL_MS = 10;

static bool lastState[LIMIT_COUNT];
static unsigned long lastPoll = 0;

bool limitActive(LimitId id) {
  return digitalRead(PINS[id]) == LIMIT_ACTIVE_LEVEL;
}

void sensorsInit() {
  for (uint8_t i = 0; i < LIMIT_COUNT; i++) {
    pinMode(PINS[i], INPUT_PULLUP);
    lastState[i] = limitActive((LimitId)i);
  }
  Serial.println(F("[SENSOR] Limit switches ready"));
}

void sensorsUpdate() {
  unsigned long now = millis();
  if (now - lastPoll < POLL_MS) return;
  lastPoll = now;

  for (uint8_t i = 0; i < LIMIT_COUNT; i++) {
    bool state = limitActive((LimitId)i);
    if (state != lastState[i]) {
      lastState[i] = state;
      Serial.print(F("[SENSOR] Limit "));
      Serial.print(NAMES[i]);
      Serial.println(state ? F(": ACTIVE") : F(": released"));
    }
  }
}

void sensorsPrintStatus() {
  Serial.print(F("[SENSOR] limits (1 = active): "));
  for (uint8_t i = 0; i < LIMIT_COUNT; i++) {
    Serial.print(NAMES[i]);
    Serial.print('=');
    Serial.print(limitActive((LimitId)i) ? 1 : 0);
    Serial.print(i < LIMIT_COUNT - 1 ? ' ' : '\n');
  }
}