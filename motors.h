#pragma once
#include <Arduino.h>

#ifndef PREFIJO
#define PREFIJO "BC"
#endif

#define NUM_MOTORES 4

void motoresInit(const byte* stepPins, const byte* dirPins, const char* const* keys);
bool motorMover(byte m, long pasos, long sps = 0);   // relativo; sps = velocidad propia (0 = la global)
void motoresActualizar();                   // llamar en cada loop()
void motoresParar();                        // STOP: cancela todo y limpia la pausa
void motoresPausa(bool pausa);              // PAUSE / PLAY
void motoresVelocidad(long sps);            // pasos por segundo
long motoresVelocidadActual();
long motorPos(byte m);