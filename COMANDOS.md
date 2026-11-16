# Control del brazo robotico

## Conexion

- Conecta el monitor serial a **115200 baudios**.
- Envia un comando por linea. El firmware acepta fin de linea `LF` o `CRLF`;
  si no se envia fin de linea, procesa el comando despues de 120 ms sin datos.
- Los comandos no distinguen mayusculas de minusculas y se ignoran espacios,
  tabulaciones y el signo `=`.

## Conexion de motores y finales

| Eje | STEP | DIR | Final de carrera |
|---|---:|---:|---:|
| X | 2 | 5 | 11 |
| Y | 3 | 6 | 10 |
| A | 4 | 7 | A2 |
| Z | 12 | 13 | A3 |

Los finales se configuran como `INPUT_PULLUP` y el firmware considera **HIGH**
como activado. Comprueba que el cableado coincida con esta logica antes de
iniciar una calibracion.

## Calibracion al encender

Al encender o reiniciar, el brazo referencia automaticamente los ejes en este
orden: **Z, X, A, Y**. Cada eje busca su final de carrera por separado, hasta
un maximo de 50000 pasos. Despues de referenciar los cuatro, se mueve a la
posicion inicial:

| Eje | Posicion inicial |
|---|---:|
| X | 8000 |
| Y | 5500 |
| A | 1200 |
| Z | 0 |

X, Y y Z se mueven primero; A se mueve despues de una pausa de 800 ms. Al
terminar, el control queda esperando en `IDLE`. La calibracion automatica solo
se inicia al reiniciar la placa; no existe un comando `CALIBRAR`.

## Uso recomendado

1. Enciende o reinicia la placa y espera a que termine el homing y el movimiento
   a la posicion inicial.
2. Envia `POS` para confirmar que el estado sea `IDLE` y que las posiciones
   sean las esperadas.
3. Envia `INICIAR` (o `R`) para ejecutar el recorrido.
4. Envia `P` o `STOP` para detener el movimiento. Tras detener, vuelve a
   referenciar cada eje antes de iniciar otro recorrido.

## Comandos

| Comando | Funcion |
|---|---|
| `R`, `START` o `INICIAR` | Inicia el recorrido automatico desde la posicion inicial. Requiere que los cuatro ejes esten referenciados y el control este en `IDLE`. |
| `P` o `STOP` | Detiene inmediatamente los motores y borra todas las referencias. Antes de volver a mover o iniciar el recorrido, hay que referenciar de nuevo los ejes. |
| `POS`, `POSICION` o `?` | Muestra el estado y la posicion actual de X, Y, A y Z. |
| `X`, `Y`, `A` o `Z` | Busca el final de carrera y establece el cero del eje indicado. Usalo despues de `STOP` para volver a referenciar ejes. |
| `X500`, `Y500`, `A500` o `Z1000` | Mueve el eje a la posicion indicada en pasos desde su cero. Solo acepta valores enteros no negativos; el destino se limita al recorrido maximo del eje. |

Las letras de eje solas buscan su final de carrera incluso si ya estaban
referenciadas; el nuevo movimiento invalida la referencia previa hasta que
encuentre el interruptor.

## Recorrido de `INICIAR`

Cada posicion se mantiene durante 2 segundos. Las coordenadas estan expresadas
en pasos logicos desde los finales de carrera.

| Paso | X | Y | A | Z | Accion del servo |
|---|---:|---:|---:|---:|---|
| 1 | 8000 | 5500 | 1200 | 41000 | Cierra a 140 grados |
| 1.1 | 8000 | 5500 | 1200 | 0 | Sin cambio |
| 1.2 | 16000 | 5500 | 1200 | 0 | Sin cambio |
| 1.3 | 16000 | 5500 | 1200 | 41000 | Abre a 0 grados |
| 1.4 | 16000 | 5500 | 1200 | 0 | Sin cambio |

Al terminar, vuelve a la posicion inicial descrita arriba. Cuando llega, queda
en espera; envia `INICIAR` o `R` para repetir el recorrido.
El servo se abre a 0 grados al encender si el PCA9685 esta disponible, se cierra
en la posicion 1 y vuelve a abrir en la posicion 1.3. No hay un comando serial
independiente para controlar el servo.

## Velocidad y movimiento

La busqueda del cero y los movimientos normales usan el mismo generador de
pasos a velocidad constante: **5000 pasos/s configurados para cada eje**, sin
rampa de aceleracion. Los ejes que tienen destinos distintos en una posicion
del recorrido se mueven simultaneamente. La velocidad efectiva simultanea
depende del microcontrolador, los drivers, la alimentacion y la carga mecanica.
La velocidad constante produce arranque y parada sin rampa; prueba primero
movimientos cortos y detente con `P` si un motor pierde pasos, golpea o se traba.

`POS` puede consultarse bajo demanda. El firmware tambien envia posiciones
cada 250 ms cuando se mueven X, Y o A sin que Z este en movimiento; pausa ese
envio durante el movimiento de Z para mantener regular su generacion de pasos.
Los movimientos individuales informan el eje y la posicion final en un evento
`EVT,MOVE_DONE`.

Los limites maximos configurados son X=30450, Y=11834, A=3426 y Z=47829 pasos.
Estos limites no sustituyen los finales de carrera ni una comprobacion de que
la estructura mecanica tenga espacio libre.

## Mensajes seriales

- `STATE,<estado>`: cambio o consulta del estado.
- `EVT,<evento>,<eje>,<valor>`: calibracion, movimiento o evento del recorrido.
- `POS,X=<pasos>,Y=<pasos>,A=<pasos>,Z=<pasos>`: posiciones actuales.
- `ERR,<codigo>`: comando rechazado o error.
- `CMD,<comando>`: eco del comando recibido.
- `INFO,<texto>`: informacion inicial.

Codigos de error habituales:

- `NOT_READY`: falta referenciar algun eje o el control no esta en espera.
- `BUSY`: hay otra operacion en curso.
- `NOT_REFERENCED,<eje>`: ese eje debe referenciarse antes de moverlo.
- `HOMING_LIMIT_NOT_FOUND,<eje>`: el eje supero 50000 pasos sin detectar su final.
- `SERVO_NOT_FOUND` o `SERVO_FAILED`: el PCA9685 no fue detectado o no acepto
  la orden del servo.
- `RESET_REQUIRED`: hubo un error de calibracion; revisa el final de carrera y
  reinicia la placa.

Despues de `RESET_REQUIRED`, el firmware no acepta movimientos ni `STOP`;
reinicia la placa una vez revisado el interruptor y el cableado.
