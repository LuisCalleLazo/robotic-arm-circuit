#include "sensors.h"

static const byte* pines;
static const int*  hacia;
static byte nivel;

void sensoresInit(const byte* pinesLimite, const int* haciaSensor, byte nivelActivo, byte cantidad) {
  pines = pinesLimite;
  hacia = haciaSensor;
  nivel = nivelActivo;
  for (byte i = 0; i < cantidad; i++) pinMode(pines[i], INPUT_PULLUP);
}

bool sensorActivo(byte m) {
  return digitalRead(pines[m]) == nivel;
}

// Solo bloquea si el motor avanza HACIA el sensor; alejarse siempre se permite
bool sensorBloquea(byte m, int sentido) {
  return sentido == hacia[m] && sensorActivo(m);
}