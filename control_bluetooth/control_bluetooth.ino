// ============================================================
//  CONTROL BLUETOOTH — ROBOT 3 RUEDAS
//  2 ruedas motrices delanteras + 1 rueda loca trasera
//
//  COMANDOS:
//    w → adelante
//    s → atrás
//    a → girar izquierda (counter-rotation)
//    d → girar derecha   (counter-rotation)
//    z → parar / reanudar (toggle)
//
//  El robot mantiene el último comando recibido hasta que
//  llegue uno nuevo. Así no hace falta mantener pulsado.
// ============================================================

#include <SoftwareSerial.h>
SoftwareSerial BT(10, 11); // RX, TX

#define LOG(x)   { Serial.print(x);   BT.print(x);   }
#define LOGLN(x) { Serial.println(x); BT.println(x); }

// ── MOTORES (L298N) ───────────────────────────────────────────
const int PIN_IN1 = 2;
const int PIN_E1  = 3;
const int PIN_IN2 = 4;
const int PIN_E2  = 5;

// ── VELOCIDADES ───────────────────────────────────────────────
const int VEL_LINEAL = 200;  // adelante / atrás
const int VEL_GIRO   = 180;  // giro counter-rotation

// ── ESTADO ───────────────────────────────────────────────────
char  ultimoComando = 'z';   // empieza parado
bool  parado        = true;

// ============================================================
//  MOTORES
// ============================================================

void motorDer(int vel) {
  vel = constrain(vel, -255, 255);
  if      (vel > 0) { digitalWrite(PIN_IN1, HIGH); analogWrite(PIN_E1,  vel); }
  else if (vel < 0) { digitalWrite(PIN_IN1, LOW);  analogWrite(PIN_E1, -vel); }
  else              { digitalWrite(PIN_IN1, HIGH);  analogWrite(PIN_E1,    0); }
}

void motorIzq(int vel) {
  vel = constrain(vel, -255, 255);
  if      (vel > 0) { digitalWrite(PIN_IN2, HIGH); analogWrite(PIN_E2,  vel); }
  else if (vel < 0) { digitalWrite(PIN_IN2, LOW);  analogWrite(PIN_E2, -vel); }
  else              { digitalWrite(PIN_IN2, HIGH);  analogWrite(PIN_E2,    0); }
}

void detener() {
  analogWrite(PIN_E1, 0);
  analogWrite(PIN_E2, 0);
}

// ============================================================
//  MOVIMIENTOS
// ============================================================

void adelante() {
  motorDer(VEL_LINEAL);
  motorIzq(VEL_LINEAL);
}

void atras() {
  motorDer(-VEL_LINEAL);
  motorIzq(-VEL_LINEAL);
}

// Giro izquierda: rueda izq atrás, rueda der adelante
void girarIzquierda() {
  motorDer( VEL_GIRO);
  motorIzq(-VEL_GIRO);
}

// Giro derecha: rueda der atrás, rueda izq adelante
void girarDerecha() {
  motorDer(-VEL_GIRO);
  motorIzq( VEL_GIRO);
}

// ============================================================
//  EJECUTAR COMANDO ACTUAL
// ============================================================

void ejecutarComando() {
  if (parado) {
    detener();
    return;
  }

  switch (ultimoComando) {
    case 'w': adelante();       break;
    case 's': atras();          break;
    case 'a': girarIzquierda(); break;
    case 'd': girarDerecha();   break;
    default:  detener();        break;
  }
}

// ============================================================
//  SETUP
// ============================================================

void setup() {
  Serial.begin(9600);
  BT.begin(9600);

  pinMode(PIN_IN1, OUTPUT); pinMode(PIN_E1, OUTPUT);
  pinMode(PIN_IN2, OUTPUT); pinMode(PIN_E2, OUTPUT);

  detener();

  LOGLN("Control BT listo.");
  LOGLN("w=adelante | s=atras | a=izquierda | d=derecha | z=parar/reanudar");
}

// ============================================================
//  LOOP
// ============================================================

void loop() {
  // Leer comando entrante por BT o Serial (útil para depurar
  // desde el monitor serie sin necesidad del módulo BT)
  char cmd = 0;
  if (BT.available())     cmd = BT.read();
  if (Serial.available()) cmd = Serial.read();

  if (cmd != 0) {
    // Ignorar saltos de línea o retornos de carro que algunos
    // terminales envían automáticamente tras cada comando
    if (cmd == '\n' || cmd == '\r') {
      // no hacer nada
    }
    else if (cmd == 'z') {
      // Toggle parar / reanudar
      parado = !parado;
      if (parado) {
        LOGLN(">> PARADO");
        detener();
      } else {
        LOG(">> REANUDANDO: ");
        LOGLN(ultimoComando);
      }
    }
    else if (cmd == 'w' || cmd == 's' || cmd == 'a' || cmd == 'd') {
      ultimoComando = cmd;
      LOG(">> Comando: ");
      LOGLN(ultimoComando);
    }
    else {
      LOG(">> Comando desconocido: ");
      LOGLN(cmd);
    }
  }

  // Ejecutar siempre el estado actual (aunque no haya comando nuevo)
  ejecutarComando();

  delay(10);
}
