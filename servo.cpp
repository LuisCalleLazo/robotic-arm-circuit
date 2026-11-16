#include "servo.h"

// Half-period of the software I2C clock. Increase if the bus is unreliable.
#define I2C_DELAY_US 5

static bool servoReady = false;

// ==================== SOFTWARE I2C ====================
static void i2cRelease(uint8_t pin) { pinMode(pin, INPUT_PULLUP); }
static void i2cLow(uint8_t pin) { digitalWrite(pin, LOW); pinMode(pin, OUTPUT); }
static void i2cDelay() { delayMicroseconds(I2C_DELAY_US); }

static void i2cStart() {
  i2cRelease(SERVO_SDA_PIN);
  i2cRelease(SERVO_SCL_PIN);
  i2cDelay();
  i2cLow(SERVO_SDA_PIN);
  i2cDelay();
  i2cLow(SERVO_SCL_PIN);
}

static void i2cStop() {
  i2cLow(SERVO_SDA_PIN);
  i2cDelay();
  i2cRelease(SERVO_SCL_PIN);
  i2cDelay();
  i2cRelease(SERVO_SDA_PIN);
  i2cDelay();
}

// Sends one byte and returns true if the slave acknowledged it.
static bool i2cWriteByte(byte data) {
  for (byte i = 0; i < 8; i++) {
    if (data & 0x80) i2cRelease(SERVO_SDA_PIN); else i2cLow(SERVO_SDA_PIN);
    i2cDelay();
    i2cRelease(SERVO_SCL_PIN);
    i2cDelay();
    i2cLow(SERVO_SCL_PIN);
    data <<= 1;
  }
  i2cRelease(SERVO_SDA_PIN);
  i2cDelay();
  i2cRelease(SERVO_SCL_PIN);
  i2cDelay();
  bool ack = (digitalRead(SERVO_SDA_PIN) == LOW);
  i2cLow(SERVO_SCL_PIN);
  return ack;
}

// ==================== PCA9685 ====================
static bool pcaWriteRegister(byte reg, byte value) {
  i2cStart();
  bool ok = i2cWriteByte(PCA9685_ADDR << 1);
  if (ok) ok = i2cWriteByte(reg);
  if (ok) ok = i2cWriteByte(value);
  i2cStop();
  return ok;
}

// Puts the PCA9685 in sleep, sets the prescaler (about 50 Hz for servos),
// wakes it up and enables register auto-increment.
static bool pcaConfigure() {
  if (!pcaWriteRegister(0x00, 0x10)) return false;  // MODE1: sleep
  if (!pcaWriteRegister(0xFE, 121)) return false;   // PRESCALE: 50 Hz
  delay(5);
  if (!pcaWriteRegister(0x00, 0x20)) return false;  // MODE1: wake, auto-increment
  delay(5);
  if (!pcaWriteRegister(0x00, 0xA0)) return false;  // MODE1: restart
  delay(10);
  return true;
}

// ==================== PUBLIC API ====================
bool servoInit() {
  i2cRelease(SERVO_SDA_PIN);
  i2cRelease(SERVO_SCL_PIN);
  delay(100);

  i2cStart();
  bool detected = i2cWriteByte(PCA9685_ADDR << 1);
  i2cStop();

  servoReady = detected && pcaConfigure();
  return servoReady;
}

bool servoIsAvailable() {
  return servoReady;
}

bool servoMove(int angle) {
  if (!servoReady) return false;

  angle = constrain(angle, 0, 180);
  // 123..492 ticks of the 4096-step PWM cycle correspond to 0..180 degrees.
  // Tune these two numbers if your servo range does not match.
  int pulse = map(angle, 0, 180, 123, 492);

  byte reg = 0x06 + 4 * SERVO_CHANNEL;  // LEDn_ON_L
  if (!pcaWriteRegister(reg, 0)) return false;               // ON low
  if (!pcaWriteRegister(reg + 1, 0)) return false;           // ON high
  if (!pcaWriteRegister(reg + 2, pulse & 0xFF)) return false;  // OFF low
  if (!pcaWriteRegister(reg + 3, pulse >> 8)) return false;    // OFF high
  return true;
}