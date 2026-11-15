#include "servo.h"

// PCA9685 over software I2C
#define PIN_SCL A0
#define PIN_SDA A1
#define PCA_ADDR 0x40
#define SERVO_CHANNEL 0

static bool available = false;

static void releaseLine(int pin) { pinMode(pin, INPUT_PULLUP); }
static void pullLow(int pin) { digitalWrite(pin, LOW); pinMode(pin, OUTPUT); }

static void i2cStart() {
  releaseLine(PIN_SDA); releaseLine(PIN_SCL); delayMicroseconds(5);
  pullLow(PIN_SDA); delayMicroseconds(5); pullLow(PIN_SCL);
}

static void i2cStop() {
  pullLow(PIN_SDA); delayMicroseconds(5); releaseLine(PIN_SCL);
  delayMicroseconds(5); releaseLine(PIN_SDA); delayMicroseconds(5);
}

static bool i2cSendByte(byte data) {
  for (byte i = 0; i < 8; i++) {
    if (data & 0x80) releaseLine(PIN_SDA); else pullLow(PIN_SDA);
    delayMicroseconds(5); releaseLine(PIN_SCL); delayMicroseconds(5);
    pullLow(PIN_SCL); data <<= 1;
  }
  releaseLine(PIN_SDA); delayMicroseconds(5); releaseLine(PIN_SCL);
  delayMicroseconds(5);
  bool ack = (digitalRead(PIN_SDA) == LOW);
  pullLow(PIN_SCL);
  return ack;
}

static bool pcaWriteRegister(byte reg, byte value) {
  i2cStart();
  bool ok = i2cSendByte(PCA_ADDR << 1);
  if (ok) ok = i2cSendByte(reg);
  if (ok) ok = i2cSendByte(value);
  i2cStop();
  return ok;
}

static bool pcaConfigure() {
  if (!pcaWriteRegister(0x00, 0x10)) return false;   // sleep
  if (!pcaWriteRegister(0xFE, 121)) return false;    // prescaler (~50 Hz)
  delay(5);
  if (!pcaWriteRegister(0x00, 0x20)) return false;   // wake, auto-increment
  delay(5);
  if (!pcaWriteRegister(0x00, 0xA0)) return false;   // restart
  delay(10);
  return true;
}

bool servoAvailable() { return available; }

bool servoMove(int angle) {
  if (!available) {
    Serial.println(F("[SERVO] Not available, move skipped"));
    return false;
  }

  angle = constrain(angle, 0, 180);
  int pulse = map(angle, 0, 180, 123, 492);
  byte reg = 0x06 + 4 * SERVO_CHANNEL;

  bool ok = pcaWriteRegister(reg, 0);
  if (ok) ok = pcaWriteRegister(reg + 1, 0);
  if (ok) ok = pcaWriteRegister(reg + 2, pulse & 0xFF);
  if (ok) ok = pcaWriteRegister(reg + 3, pulse >> 8);

  Serial.print(F("[SERVO] Move to "));
  Serial.print(angle);
  Serial.println(ok ? F(" deg OK") : F(" deg FAILED"));
  return ok;
}

void servoInit() {
  releaseLine(PIN_SDA);
  releaseLine(PIN_SCL);
  delay(100);

  i2cStart();
  bool detected = i2cSendByte(PCA_ADDR << 1);
  i2cStop();

  if (detected && pcaConfigure()) {
    available = true;
    Serial.println(F("[SERVO] PCA9685 detected, servo on channel 0"));
    servoMove(SERVO_OPEN_ANGLE);
  } else {
    Serial.println(F("[SERVO] WARNING: PCA9685 not detected, route will run without servo"));
  }
}