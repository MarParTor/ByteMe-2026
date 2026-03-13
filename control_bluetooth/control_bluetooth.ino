
// ============================================================
//  CONTROL BLUETOOTH — ROBOT 3 RUEDAS
//  w=adelante | s=atrás | a=izq | d=der | z=parar
//
//  SI NO RESPONDE POR BT: cambia BT_BAUD a 38400 (HC-05)
//                         o déjalo en 9600 (HC-06)
// ============================================================

#include <SoftwareSerial.h>
SoftwareSerial BT(10, 11);

#define BT_BAUD  9600


/* Feliz */
#define FORWARD  'w'
#define BACKWARD 's'
#define LEFT     'a'
#define RIGHT    'd'
#define STOP     'z'


/* Enfadado 
#define FORWARD  's'
#define BACKWARD 'w'
#define LEFT     'a'
#define RIGHT    'd'
#define STOP     'z'
*/
#define PIN_IN1 2
#define PIN_E1  3
#define PIN_IN2 5
#define PIN_E2  4

#define VEL_LINEAL 255
#define VEL_GIRO   255

// ============================================================
//  MOTORES
// ============================================================

void motorIzq(int vel) {
  vel = constrain(vel, -255, 255);
  if      (vel > 0) { digitalWrite(PIN_IN1, HIGH); analogWrite(PIN_E1,  vel); }
  else if (vel < 0) { digitalWrite(PIN_IN1, LOW);  analogWrite(PIN_E1, -vel); }
  else              { digitalWrite(PIN_IN1, HIGH);  analogWrite(PIN_E1,    0); }
}

void motorDer(int vel) {
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

void adelante()       { motorDer( VEL_LINEAL); motorIzq( VEL_LINEAL); }
void atras()          { motorDer(-VEL_LINEAL); motorIzq(-VEL_LINEAL); }
void girarIzquierda() { motorDer( -VEL_GIRO);   motorIzq(VEL_GIRO);   }
void girarDerecha()   { motorDer(VEL_GIRO);   motorIzq( -VEL_GIRO);   }

// ============================================================
//  EJECUTAR COMANDO
// ============================================================

void executeCommand(char cmd) {
  if (cmd == '\n' || cmd == '\r') return;

  switch (cmd) {
    case FORWARD:   adelante();       break;
    case BACKWARD:  atras();          break;
    case LEFT:      girarIzquierda(); break;
    case RIGHT:     girarDerecha();   break;
    case STOP:      detener();        break;
    default: return;
  }

  BT.print(">> "); BT.println(cmd);
  Serial.print(">> "); Serial.println(cmd);
}

// ============================================================
//  SETUP
// ============================================================

void setup() {
  Serial.begin(9600);
  BT.begin(BT_BAUD);

  pinMode(PIN_IN1, OUTPUT); pinMode(PIN_E1, OUTPUT);
  pinMode(PIN_IN2, OUTPUT); pinMode(PIN_E2, OUTPUT);

  detener();
  delay(1000);

  BT.println("Control BT listo. w/s/a/d/z");
  Serial.println("Control BT listo. w/s/a/d/z");
}

// ============================================================
//  LOOP
// ============================================================

void loop() {
  if (BT.available())     executeCommand(BT.read());
  if (Serial.available()) executeCommand(Serial.read());
}