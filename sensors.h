#pragma once
#include <Arduino.h>

void sensoresInit(const byte* pinesLimite, const int* haciaSensor, byte nivelActivo, byte cantidad);
bool sensorActivo(byte m);                  // true si el limite del motor m esta activado
bool sensorBloquea(byte m, int sentido);    // true si ir en 'sentido' choca con un limite ya activado