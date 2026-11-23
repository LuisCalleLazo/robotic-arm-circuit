#pragma once
#include <Arduino.h>

#ifndef PREFIJO
#define PREFIJO "BC"
#endif

void servoConfigurar(int minAngulo, int maxAngulo, int velMaxima, int velDefecto);   // llamar antes de servoInit
bool servoInit(byte pinSCL, byte pinSDA, byte direccion, byte canal, const char* key);
bool servoMover(int angulo, int gradosPorSeg = -1);   // absoluto 0..180; vel propia (-1 = la global, 0 = la maxima permitida)
void servoActualizar();                     // llamar en cada loop()
void servoParar();
void servoPausa(bool pausa);
void servoVelocidad(int gradosPorSeg);      // 0 = la maxima permitida
int  servoVelocidadActual();
int  servoAngulo();