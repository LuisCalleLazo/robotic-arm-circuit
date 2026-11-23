# BRAZO - Control por Serial

Serial a **115200 baud**, comandos terminados en salto de linea. No distingue mayusculas.
Sin telemetria: solo ejecuta comandos y responde eventos cortos que empiezan con `BC,`.

## Estructura

| Archivo | Funcion |
|---|---|
| `BRAZO.ino` | Configuracion (keys, pines), parser serial, loop |
| `motors.h/.cpp` | Motores X, Y, A, Z (pasos no bloqueantes, varios a la vez) |
| `sensors.h/.cpp` | Limites: bloquean solo el avance hacia el sensor |
| `servo.h/.cpp` | Servo por PCA9685 (I2C por software), 1 grado a la vez segun velocidad |

La carpeta del sketch debe llamarse `BRAZO`.

## Comandos

| Comando | Accion |
|---|---|
| `BCX500` / `BCX-500` | Motor X avanza / retrocede 500 pasos (relativo) |
| `BCY..` `BCA..` `BCZ..` | Igual para Y, A, Z |
| `BCX500:2000` | 500 pasos a 2000 pasos/seg (solo este movimiento) |
| `BCS120` | Servo a 120 grados (absoluto, 0..180) |
| `BCS120:30` | Servo a 120 grados a 30 grados/seg (`:0` = velocidad maxima permitida) |
| `BCX500:2000 BCY-300 BCS120:30` | Varios a la vez; la velocidad es opcional en cada uno |
| `BCSTOP` | Para todo y borra lo pendiente |
| `BCPAUSE` / `BCPLAY` | Congela todo / continua donde quedo |
| `BCV7000` | Velocidad por defecto de motores (100..10000 pasos/seg) |
| `BCVS90` | Velocidad por defecto del servo (1..`SERVO_VEL_MAX` grados/seg, 0 = la maxima) |
| `BC?` | Pausa, velocidades y posicion de cada eje |

## Reglas

- Si un token es invalido, la linea completa se descarta y no se mueve nada.
- Si un motor ya esta en movimiento y le mandas otro movimiento: `BC,ERR,BUSY,BCX`.
- Los sensores se revisan antes de **cada paso**: si el motor avanza hacia su sensor y este se activa, se detiene (`BC,EVT,SENSOR,...`). Alejarse siempre se permite.
- Con `BCPAUSE` activo, los comandos nuevos quedan en espera hasta `BCPLAY`.

## Seguridad del servo

- **No se mueve al abrir/cerrar el serial ni al reiniciar el Arduino.** Si el PCA9685 ya estaba configurado, no se toca y se lee en que angulo esta; si no, no se manda ningun angulo hasta el primer comando (ese primero fija el reposo en `SERVO_MIN`).
- Nunca hay saltos: toda velocidad se limita a `SERVO_VEL_MAX` (por defecto 180 grados/seg), tambien `:0` y `BCVS0`.
- El angulo se limita a `SERVO_MIN`..`SERVO_MAX` (arriba de `BRAZO.ino`). Si el servo topa mecanicamente en los extremos, sube el minimo y baja el maximo unos grados (ej. 5 y 175).
- Velocidad por defecto: `SERVO_VEL_DEF` (60 grados/seg).
- Al abrir el serial, el Arduino se reinicia (DTR). Para que el bus I2C no genere ruido durante ese reinicio, SDA y SCL (A1 y A0) deben tener pull-up a 5V (la mayoria de los modulos PCA9685 ya los traen). Alimenta el servo con fuente propia (5-6V, con corriente suficiente) y GND comun.

## Respuestas

```
BC,INFO,LISTO
BC,INFO,SERVO=1
BC,EVT,INICIO,BCX,500
BC,EVT,FIN,BCX,500          (pos final)
BC,EVT,SENSOR,BCX,320       (se detuvo por limite)
BC,EVT,STOP | PAUSE | PLAY
BC,CFG,VEL,5000
BC,ERR,BUSY,BCX
BC,ERR,SENSOR_BLOQUEADO,BCX
BC,ERR,UNKNOWN,<token>
BC,ERR,BADSPEED,<token>
BC,ERR,SERVO_NOT_FOUND | SERVO_FAILED
```

## Configuracion (arriba de `BRAZO.ino`)

Keys, pines, `HACIA_SENSOR[]` (sentido que va hacia cada sensor), `NIVEL_ACTIVO` y datos del PCA9685.