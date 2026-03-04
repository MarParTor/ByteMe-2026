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
// ============================================================

#include <SoftwareSerial.h>
SoftwareSerial BT(10, 11); // RX, TX

// ── COMANDOS ─────────────────────────────────────────────────
#define FORWARD  'w'
#define BACKWARD 's'
#define LEFT     'a'
#define RIGHT    'd'
#define TOGGLE   'z'

// ── MOTORES (L298N) ───────────────────────────────────────────
#define PIN_IN1 2
#define PIN_E1  3
#define PIN_IN2 4
#define PIN_E2  5

// ── VELOCIDADES ───────────────────────────────────────────────
#define VEL_LINEAL 200
#define VEL_GIRO   180

// ── ESTADO ───────────────────────────────────────────────────
char ultimoComando = TOGGLE;  // empieza parado
bool parado        = true;

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

void girarIzquierda() {
  motorDer( VEL_GIRO);
  motorIzq(-VEL_GIRO);
}

void girarDerecha() {
  motorDer(-VEL_GIRO);
  motorIzq( VEL_GIRO);
}

// ============================================================
//  EJECUTAR COMANDO
// ============================================================

void executeCommand(char command) {
  // Ignorar saltos de línea que algunos terminales envían
  if (command == '\n' || command == '\r') return;

  switch (command) {
    case FORWARD:
      if (!parado) adelante();
      ultimoComando = FORWARD;
      Serial.println(">> Adelante");
      BT.println(">> Adelante");
      break;

    case BACKWARD:
      if (!parado) atras();
      ultimoComando = BACKWARD;
      Serial.println(">> Atras");
      BT.println(">> Atras");
      break;

    case LEFT:
      if (!parado) girarIzquierda();
      ultimoComando = LEFT;
      Serial.println(">> Izquierda");
      BT.println(">> Izquierda");
      break;

    case RIGHT:
      if (!parado) girarDerecha();
      ultimoComando = RIGHT;
      Serial.println(">> Derecha");
      BT.println(">> Derecha");
      break;

    case TOGGLE:
      parado = !parado;
      if (parado) {
        detener();
        Serial.println(">> PARADO");
        BT.println(">> PARADO");
      } else {
        // Reanudar ejecutando el último comando recibido
        executeCommand(ultimoComando);
        Serial.println(">> REANUDANDO");
        BT.println(">> REANUDANDO");
      }
      break;

    default:
      Serial.print(">> Comando desconocido: ");
      Serial.println(command);
      BT.print(">> Comando desconocido: ");
      BT.println(command);
      break;
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

  Serial.println("Control BT listo.");
  Serial.println("w=adelante | s=atras | a=izquierda | d=derecha | z=parar/reanudar");
  BT.println("Control BT listo.");
  BT.println("w=adelante | s=atras | a=izquierda | d=derecha | z=parar/reanudar");
}

// ============================================================
//  LOOP
// ============================================================

void loop() {
  if (BT.available()) {
    char command = BT.read();
    executeCommand(command);
  }

  if (Serial.available()) {
    char command = Serial.read();
    executeCommand(command);
  }
}
