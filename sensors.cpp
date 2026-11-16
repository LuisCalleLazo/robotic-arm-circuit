#include "sensors.h"

void sensorsInit() {
  pinMode(LIMIT_X_PIN, INPUT_PULLUP);
  pinMode(LIMIT_Y_PIN, INPUT_PULLUP);
  pinMode(LIMIT_A_PIN, INPUT_PULLUP);
  pinMode(LIMIT_Z_PIN, INPUT_PULLUP);
}

bool sensorsLimitActive(uint8_t pin) {
  return digitalRead(pin) == LIMIT_ACTIVE_LEVEL;
}