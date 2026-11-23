#include "motors.h"
#include "sensors.h"
#include "servo.h"

// =====================================================================
//                        CONFIGURACION (EDITAR AQUI)
// =====================================================================

// ---- KEYS de movimiento (pueden tener N caracteres) ----
//                                      X      Y      A      Z
const char* const KEY_MOTOR[NUM_MOTORES] = {"BCX", "BCY", "BCA", "BCZ"};
const char* const KEY_SERVO     = "BCS";      // BCS120:30 -> servo a 120 grados a 30 grados/seg

// ---- KEYS de control ----
const char* const KEY_STOP      = "BCSTOP";   // para todo y cancela lo pendiente
const char* const KEY_PAUSE     = "BCPAUSE";  // congela todo, conserva lo pendiente
const char* const KEY_PLAY      = "BCPLAY";   // continua tras PAUSE
const char* const KEY_ESTADO    = "BC?";      // pide estado y posiciones
const char* const KEY_VEL       = "BCV";      // BCV7000  -> pasos/seg por defecto de los motores
const char* const KEY_VEL_SERVO = "BCVS";     // BCVS90   -> grados/seg por defecto del servo (0 = maxima permitida)

// ---- Pines motores (X, Y, A, Z) ----
const byte STEP_PIN[NUM_MOTORES]  = {2, 3, 4, 12};
const byte DIR_PIN[NUM_MOTORES]   = {5, 6, 7, 13};

// ---- Sensores de limite (X, Y, A, Z) ----
const byte LIMIT_PIN[NUM_MOTORES] = {11, 10, A2, A3};
// Sentido que va HACIA el sensor: +1 pasos positivos, -1 pasos negativos
const int  HACIA_SENSOR[NUM_MOTORES] = {+1, +1, -1, -1};
const byte NIVEL_ACTIVO = HIGH;               // nivel que lee el sensor al chocar

// ---- Servo PCA9685 (I2C por software) ----
const byte PIN_SCL = A0;
const byte PIN_SDA = A1;
const byte PCA_ADDR = 0x40;
const byte CANAL_SERVO = 0;

// ---- Seguridad del servo (PDI-6221MG-180) ----
const int SERVO_MIN = 0;          // limite minimo en grados (sube a ~5 si topa mecanicamente)
const int SERVO_MAX = 180;        // limite maximo en grados
const int SERVO_VEL_MAX = 180;    // grados/seg maximos permitidos (BCS..:0 y BCVS0 usan este valor)
const int SERVO_VEL_DEF = 60;     // grados/seg cuando no se indica velocidad

// =====================================================================

bool pausado = false;

bool numero(const char* s, long& v) {
  bool neg = false;
  if (*s == '-') { neg = true; s++; } else if (*s == '+') s++;
  if (!*s) return false;
  long r = 0; byte n = 0;
  while (*s) {
    if (*s < '0' || *s > '9' || ++n > 9) return false;
    r = r * 10 + (*s - '0'); s++;
  }
  v = neg ? -r : r;
  return true;
}

// true si 'tok' = key + numero
bool conKey(const char* tok, const char* key, long& v) {
  size_t k = strlen(key);
  if (strncmp(tok, key, k) != 0) return false;
  return numero(tok + k, v);
}

void evtSimple(const __FlashStringHelper* tipo) {
  Serial.print(PREFIJO); Serial.print(F(",EVT,")); Serial.println(tipo);
}

void cfg(const __FlashStringHelper* nombre, long valor) {
  Serial.print(PREFIJO); Serial.print(F(",CFG,")); Serial.print(nombre);
  Serial.print(','); Serial.println(valor);
}

void detener() {
  motoresParar();
  servoParar();
  pausado = false;
  evtSimple(F("STOP"));
}

void pausar(bool p) {
  pausado = p;
  motoresPausa(p);
  servoPausa(p);
  evtSimple(p ? F("PAUSE") : F("PLAY"));
}

void estado() {
  Serial.print(PREFIJO); Serial.print(F(",STATUS,PAUSA=")); Serial.print(pausado ? 1 : 0);
  Serial.print(F(",VEL=")); Serial.print(motoresVelocidadActual());
  Serial.print(F(",VELS=")); Serial.println(servoVelocidadActual());
  for (byte i = 0; i < NUM_MOTORES; i++) {
    Serial.print(KEY_MOTOR[i]); Serial.print('='); Serial.println(motorPos(i));
  }
  Serial.print(KEY_SERVO); Serial.print('='); Serial.println(servoAngulo());
}

// Una linea puede traer varios tokens separados por espacio: se arrancan todos a la vez
//   BCX500:2000 BCY-300 BCS120:30
void procesarLinea(char* linea) {
  long pasos[NUM_MOTORES], velMotor[NUM_MOTORES];
  bool hay[NUM_MOTORES];
  for (byte i = 0; i < NUM_MOTORES; i++) { hay[i] = false; velMotor[i] = -1; }
  long angulo = 0, velServo = -1; bool hayServo = false;

  char* ctx;
  for (char* tok = strtok_r(linea, " ,;\t", &ctx); tok; tok = strtok_r(NULL, " ,;\t", &ctx)) {
    long v;
    if (!strcmp(tok, KEY_STOP))   { detener(); return; }
    if (!strcmp(tok, KEY_PAUSE))  { pausar(true); continue; }
    if (!strcmp(tok, KEY_PLAY))   { pausar(false); continue; }
    if (!strcmp(tok, KEY_ESTADO)) { estado(); continue; }

    if (conKey(tok, KEY_VEL_SERVO, v)) { servoVelocidad(v); cfg(F("VELS"), servoVelocidadActual()); continue; }
    if (conKey(tok, KEY_VEL, v))       { motoresVelocidad(v); cfg(F("VEL"), motoresVelocidadActual()); continue; }

    // Velocidad propia del movimiento:  BCX500:2000   BCS120:30
    long vel = -1;
    char* dp = strchr(tok, ':');
    if (dp) {
      *dp = '\0';
      if (!numero(dp + 1, vel) || vel < 0) {
        Serial.print(PREFIJO); Serial.print(F(",ERR,BADSPEED,")); Serial.println(tok);
        return;
      }
    }

    if (conKey(tok, KEY_SERVO, v))     { angulo = constrain(v, 0L, 180L); velServo = vel; hayServo = true; continue; }

    bool ok = false;
    for (byte i = 0; i < NUM_MOTORES; i++) {
      if (conKey(tok, KEY_MOTOR[i], v)) { pasos[i] = v; velMotor[i] = vel; hay[i] = true; ok = true; break; }
    }
    if (!ok) {
      Serial.print(PREFIJO); Serial.print(F(",ERR,UNKNOWN,")); Serial.println(tok);
      return;                                   // linea invalida: no se mueve nada
    }
  }

  for (byte i = 0; i < NUM_MOTORES; i++) if (hay[i]) motorMover(i, pasos[i], velMotor[i] > 0 ? velMotor[i] : 0);
  if (hayServo) servoMover((int)angulo, (int)velServo);
}

// ---- Lector serial sin String ----
char buf[80];
byte nbuf = 0;
bool descartar = false;
unsigned long tUltimoChar = 0;

void leerSerial() {
  while (Serial.available() > 0) {
    char c = (char)Serial.read();
    tUltimoChar = millis();
    if (c == '\n' || c == '\r') {
      if (!descartar && nbuf) { buf[nbuf] = '\0'; procesarLinea(buf); }
      nbuf = 0; descartar = false;
    } else if (nbuf < sizeof(buf) - 1) {
      buf[nbuf++] = toupper(c);
    } else if (!descartar) {
      descartar = true;
      Serial.print(PREFIJO); Serial.println(F(",ERR,LINEA_LARGA"));
    }
  }
  // Si no llega salto de linea, procesa tras 150 ms de silencio
  if (nbuf && !descartar && millis() - tUltimoChar >= 150) {
    buf[nbuf] = '\0'; procesarLinea(buf); nbuf = 0;
  }
}

void setup() {
  Serial.begin(115200);
  sensoresInit(LIMIT_PIN, HACIA_SENSOR, NIVEL_ACTIVO, NUM_MOTORES);
  motoresInit(STEP_PIN, DIR_PIN, KEY_MOTOR);
  servoConfigurar(SERVO_MIN, SERVO_MAX, SERVO_VEL_MAX, SERVO_VEL_DEF);
  bool servoOk = servoInit(PIN_SCL, PIN_SDA, PCA_ADDR, CANAL_SERVO, KEY_SERVO);

  Serial.print(PREFIJO); Serial.println(F(",INFO,LISTO"));
  Serial.print(PREFIJO); Serial.print(F(",INFO,SERVO=")); Serial.println(servoOk ? 1 : 0);
}

void loop() {
  motoresActualizar();
  servoActualizar();
  leerSerial();
}