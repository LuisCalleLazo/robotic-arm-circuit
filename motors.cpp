#include "motors.h"
#include "sensors.h"

struct Motor {
  byte stepPin, dirPin;
  const char* key;
  long pos, restantes;
  int sentido;
  bool activo;
  unsigned long ultimoPaso;   // micros
  unsigned long periodo;      // us entre pasos
  bool propia;                // true = velocidad propia de este movimiento
};

static Motor M[NUM_MOTORES];
static long velocidad = 7000;
static unsigned long periodoUs = 142;
static bool pausado = false;

static void evt(const __FlashStringHelper* tipo, const char* key, long valor) {
  Serial.print(PREFIJO); Serial.print(F(",EVT,")); Serial.print(tipo);
  Serial.print(','); Serial.print(key);
  Serial.print(','); Serial.println(valor);
}

static void err(const __FlashStringHelper* tipo, const char* key) {
  Serial.print(PREFIJO); Serial.print(F(",ERR,")); Serial.print(tipo);
  Serial.print(','); Serial.println(key);
}

void motoresInit(const byte* stepPins, const byte* dirPins, const char* const* keys) {
  for (byte i = 0; i < NUM_MOTORES; i++) {
    M[i].stepPin = stepPins[i];
    M[i].dirPin = dirPins[i];
    M[i].key = keys[i];
    M[i].pos = 0;
    M[i].restantes = 0;
    M[i].sentido = 0;
    M[i].activo = false;
    M[i].ultimoPaso = 0;
    M[i].periodo = periodoUs;
    M[i].propia = false;
    pinMode(M[i].stepPin, OUTPUT);
    pinMode(M[i].dirPin, OUTPUT);
    digitalWrite(M[i].stepPin, LOW);
  }
  motoresVelocidad(velocidad);
}

void motoresVelocidad(long sps) {
  velocidad = constrain(sps, 100L, 10000L);
  periodoUs = 1000000UL / velocidad;
  for (byte i = 0; i < NUM_MOTORES; i++) if (!M[i].propia) M[i].periodo = periodoUs;
}

long motoresVelocidadActual() { return velocidad; }
long motorPos(byte m) { return M[m].pos; }

bool motorMover(byte m, long pasos, long sps) {
  Motor &k = M[m];
  if (pasos == 0) return true;
  if (k.activo) { err(F("BUSY"), k.key); return false; }

  int s = (pasos > 0) ? +1 : -1;
  if (sensorBloquea(m, s)) { err(F("SENSOR_BLOQUEADO"), k.key); return false; }

  digitalWrite(k.dirPin, s > 0 ? HIGH : LOW);
  delayMicroseconds(10);                       // DM542 necesita >= 5 us tras cambiar DIR

  k.sentido = s;
  k.propia = (sps > 0);
  k.periodo = k.propia ? 1000000UL / constrain(sps, 100L, 10000L) : periodoUs;
  k.restantes = labs(pasos);
  k.activo = true;
  k.ultimoPaso = micros();
  evt(F("INICIO"), k.key, pasos);
  return true;
}

void motoresActualizar() {
  if (pausado) return;
  unsigned long ahora = micros();

  for (byte m = 0; m < NUM_MOTORES; m++) {
    Motor &k = M[m];
    if (!k.activo) continue;
    if (ahora - k.ultimoPaso < k.periodo) continue;

    // Vigilancia del limite antes de CADA paso
    if (sensorBloquea(m, k.sentido)) {
      k.activo = false; k.restantes = 0;
      evt(F("SENSOR"), k.key, k.pos);
      continue;
    }

    digitalWrite(k.stepPin, HIGH);
    delayMicroseconds(3);                      // pulso >= 2.5 us para el DM542
    digitalWrite(k.stepPin, LOW);

    k.ultimoPaso = ahora;
    k.pos += k.sentido;
    if (--k.restantes == 0) {
      k.activo = false;
      evt(F("FIN"), k.key, k.pos);
    }
  }
}

void motoresParar() {
  for (byte m = 0; m < NUM_MOTORES; m++) { M[m].activo = false; M[m].restantes = 0; }
  pausado = false;
}

void motoresPausa(bool pausa) {
  pausado = pausa;
  if (!pausa) {
    unsigned long ahora = micros();
    for (byte m = 0; m < NUM_MOTORES; m++) M[m].ultimoPaso = ahora;
  }
}