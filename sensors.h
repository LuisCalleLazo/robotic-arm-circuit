#ifndef SENSORS_H
#define SENSORS_H

#include <Arduino.h>

// ==================== LIMIT SWITCH PINS ====================
#define LIMIT_X_PIN 11
#define LIMIT_Y_PIN 10
#define LIMIT_A_PIN A2
#define LIMIT_Z_PIN A3

// Logic level that means "switch activated".
// Pins use INPUT_PULLUP and the original wiring reports HIGH when activated.
// Change to LOW if you rewire the switches as normally-open to GND.
const uint8_t LIMIT_ACTIVE_LEVEL = HIGH;

// Configures all limit switch pins.
void sensorsInit();

// Returns true if the limit switch connected to `pin` is activated.
bool sensorsLimitActive(uint8_t pin);

#endif