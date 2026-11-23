#include "servo.h"

// ==================== I2C POR SOFTWARE (acceso directo a puertos, AVR) ====================
#define I2C_US 4   // medio periodo de reloj en us

static byte addr, canal;
static const char* keyServo;

static volatile uint8_t *ddrSDA, *portSDA, *inSDA, *ddrSCL, *portSCL;
static uint8_t mSDA, mSCL;

static bool disponible = false, activo = false, pausado = false;
static bool posConocida = false;              // false = el PCA no estaba manejando el servo
static int actual = 0, objetivo = 0;          // grados
static int minAng = 0, maxAng = 180;          // limites de seguridad
static int velMax = 180, velocidad = 60;      // grados/seg
static unsigned long intervalo = 0;           // ms por grado del movimiento actual
static unsigned long ultimo = 0;

static inline void sdaLiberar() { *ddrSDA &= ~mSDA; *portSDA |= mSDA; }
static inline void sdaBajo()    { *portSDA &= ~mSDA; *ddrSDA |= mSDA; }
static inline void sclLiberar() { *ddrSCL &= ~mSCL; *portSCL |= mSCL; }
static inline void sclBajo()    { *portSCL &= ~mSCL; *ddrSCL |= mSCL; }
static inline bool sdaLee()     { return (*inSDA & mSDA) != 0; }

static void i2cInicio() {
  sdaLiberar(); sclLiberar(); delayMicroseconds(I2C_US);
  sdaBajo(); delayMicroseconds(I2C_US); sclBajo();
}
static void i2cFin() {
  sdaBajo(); delayMicroseconds(I2C_US); sclLiberar();
  delayMicroseconds(I2C_US); sdaLiberar(); delayMicroseconds(I2C_US);
}
static bool enviarByte(byte dato) {
  for (byte i = 0; i < 8; i++) {
    if (dato & 0x80) sdaLiberar(); else sdaBajo();
    delayMicroseconds(I2C_US); sclLiberar(); delayMicroseconds(I2C_US);
    sclBajo(); dato <<= 1;
  }
  sdaLiberar(); delayMicroseconds(I2C_US); sclLiberar();
  delayMicroseconds(I2C_US); bool ack = !sdaLee();
  sclBajo(); return ack;
}
static byte leerByteUltimo() {                // lee 1 byte y responde NACK
  byte v = 0;
  sdaLiberar();
  for (byte i = 0; i < 8; i++) {
    delayMicroseconds(I2C_US); sclLiberar(); delayMicroseconds(I2C_US);
    v = (v << 1) | (sdaLee() ? 1 : 0);
    sclBajo();
  }
  sdaLiberar(); delayMicroseconds(I2C_US); sclLiberar();
  delayMicroseconds(I2C_US); sclBajo();
  return v;
}

// Una sola transaccion (el PCA9685 tiene auto-incremento activado)
static bool pcaEscribir(byte reg, const byte* datos, byte n) {
  i2cInicio();
  bool ok = enviarByte(addr << 1);
  if (ok) ok = enviarByte(reg);
  for (byte i = 0; ok && i < n; i++) ok = enviarByte(datos[i]);
  i2cFin();
  return ok;
}
static bool pcaRegistro(byte reg, byte valor) { return pcaEscribir(reg, &valor, 1); }

static bool pcaLeer(byte reg, byte &valor) {
  i2cInicio();
  bool ok = enviarByte(addr << 1);
  if (ok) ok = enviarByte(reg);
  if (ok) { i2cInicio(); ok = enviarByte((addr << 1) | 1); }   // inicio repetido
  if (ok) valor = leerByteUltimo();
  i2cFin();
  return ok;
}

static bool configurarPCA() {
  if (!pcaRegistro(0x00, 0x10)) return false;
  if (!pcaRegistro(0xFE, 121)) return false;
  delay(5);
  if (!pcaRegistro(0x00, 0x20)) return false;
  delay(5);
  if (!pcaRegistro(0x00, 0xA0)) return false;
  delay(10); return true;
}

// completo: ON y OFF; si no, solo OFF
static bool escribirAngulo(int angulo, bool completo) {
  int pulso = map(angulo, 0, 180, 123, 492);
  byte reg = 0x06 + 4 * canal;
  byte d[4] = {0, 0, (byte)(pulso & 0xFF), (byte)(pulso >> 8)};
  return completo ? pcaEscribir(reg, d, 4) : pcaEscribir(reg + 2, d + 2, 2);
}

static void evt(const __FlashStringHelper* tipo, long valor) {
  Serial.print(PREFIJO); Serial.print(F(",EVT,")); Serial.print(tipo);
  Serial.print(','); Serial.print(keyServo);
  Serial.print(','); Serial.println(valor);
}
static void err(const __FlashStringHelper* tipo) {
  Serial.print(PREFIJO); Serial.print(F(",ERR,")); Serial.println(tipo);
}

// ==================== API ====================
void servoConfigurar(int minAngulo, int maxAngulo, int velMaxima, int velDefecto) {
  minAng = constrain(minAngulo, 0, 180);
  maxAng = constrain(maxAngulo, minAng, 180);
  velMax = constrain(velMaxima, 1, 720);
  velocidad = constrain(velDefecto, 1, velMax);
}

// NO mueve el servo al arrancar/reiniciar (ej. al abrir o cerrar el serial):
//  - si el PCA9685 ya estaba configurado, no se reconfigura y se lee en que angulo esta
//  - si no, se configura pero no se manda ningun angulo hasta el primer comando
bool servoInit(byte scl, byte sda, byte direccion, byte ch, const char* key) {
  addr = direccion; canal = ch; keyServo = key;

  mSDA = digitalPinToBitMask(sda);
  ddrSDA = portModeRegister(digitalPinToPort(sda));
  portSDA = portOutputRegister(digitalPinToPort(sda));
  inSDA = portInputRegister(digitalPinToPort(sda));
  mSCL = digitalPinToBitMask(scl);
  ddrSCL = portModeRegister(digitalPinToPort(scl));
  portSCL = portOutputRegister(digitalPinToPort(scl));

  sdaLiberar(); sclLiberar();
  delay(100);
  i2cInicio();
  bool detectado = enviarByte(addr << 1);
  i2cFin();
  if (!detectado) { err(F("SERVO_NOT_FOUND")); return false; }

  byte modo = 0, pre = 0;
  bool configurado = pcaLeer(0x00, modo) && pcaLeer(0xFE, pre) && pre == 121 && !(modo & 0x10);
  if (!configurado && !configurarPCA()) { err(F("SERVO_FAILED")); return false; }

  actual = constrain(0, minAng, maxAng);       // reposo si no se conoce la posicion
  posConocida = false;
  if (configurado) {
    byte lo = 0, hi = 0;
    byte base = 0x06 + 4 * canal;
    if (pcaLeer(base + 2, lo) && pcaLeer(base + 3, hi) && !(hi & 0x10)) {
      int pulso = ((hi & 0x0F) << 8) | lo;
      if (pulso >= 100 && pulso <= 520) {
        actual = constrain(map(pulso, 123, 492, 0, 180), 0, 180);
        posConocida = true;
      }
    }
  }
  objetivo = actual;
  disponible = true;
  return true;
}

// BCS120:30 -> ir a 120 grados a 30 grados/seg (1 grado cada 1000/30 ms)
bool servoMover(int angulo, int gradosPorSeg) {
  if (!disponible) { err(F("SERVO_NOT_FOUND")); return false; }
  objetivo = constrain(angulo, minAng, maxAng);

  int vel = (gradosPorSeg < 0) ? velocidad : (gradosPorSeg == 0 ? velMax : constrain(gradosPorSeg, 1, velMax));
  intervalo = 1000UL / vel;

  if (!posConocida) {                          // primera orden: fija el angulo de reposo
    if (!escribirAngulo(actual, true)) { err(F("SERVO_FAILED")); return false; }
    posConocida = true;
  }

  evt(F("INICIO"), objetivo);
  if (objetivo == actual) { activo = false; evt(F("FIN"), objetivo); return true; }
  activo = true;
  ultimo = millis() - intervalo;
  return true;
}

void servoActualizar() {
  if (!activo || pausado) return;
  unsigned long ahora = millis();
  if (ahora - ultimo < intervalo) return;
  ultimo = ahora;

  actual += (objetivo > actual) ? 1 : -1;

  if (!escribirAngulo(actual, false)) { activo = false; err(F("SERVO_FAILED")); return; }
  if (actual == objetivo) { activo = false; evt(F("FIN"), objetivo); }
}

void servoParar() { activo = false; pausado = false; }

void servoPausa(bool pausa) {
  pausado = pausa;
  if (!pausa) ultimo = millis();
}

void servoVelocidad(int gradosPorSeg) { velocidad = (gradosPorSeg <= 0) ? velMax : constrain(gradosPorSeg, 1, velMax); }
int  servoVelocidadActual() { return velocidad; }
int  servoAngulo() { return actual; }