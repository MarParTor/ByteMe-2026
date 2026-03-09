#include <SoftwareSerial.h>
SoftwareSerial BT(10, 11);

#define LOG(x)   { Serial.print(x);   BT.print(x);   }
#define LOGLN(x) { Serial.println(x); BT.println(x); }

// SENSORES
const int sensorPins[6] = { A0, A1, A2, A3, A4, A5 };
const int NUM_SENSORS   = 6;

// MOTORES (L298N)
const int PIN_IN1 = 2;
const int PIN_E1  = 3;
const int PIN_IN2 = 4;
const int PIN_E2  = 5;

// VELOCIDADES
const int VEL_BASE     = 170;
const int VEL_MAXIMA   = 255;
const int VEL_BUSQUEDA = 150;

// UMBRAL DE NEGRO (0-100 normalizado)
const int UMBRAL_NEGRO = 50;

// CALIBRACIÓN
int  calMin[NUM_SENSORS];
int  calMax[NUM_SENSORS];
const int LECTURAS_INIT = 200;
bool calibracionLista   = false;

const float Kp = 20.0;   // sube de 5 en 5
const float Kd =  12.0;   // sube de 2 en 2
const float Ki =  1.0;   // sube de 0.5 en 0.5


int   errorAnterior  = 0;
float integrador     = 0.0;

int  ultimoError      = 0;
int  ultimoLado       = 1;
int  ladoAlPerder     = 1;
int  errorAlPerder    = 0;
bool buscandoLinea    = false;
unsigned long tInicioBusqueda = 0;
const unsigned long T_MAX_BUSQUEDA = 9000;

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

void inicializarCalibracion() {
  for (int i = 0; i < NUM_SENSORS; i++) {
    calMin[i] = 1023;
    calMax[i] = 0;
  }
}

void actualizarCalibracion(int valores[], bool soloMax) {
  for (int i = 0; i < NUM_SENSORS; i++) {
    if (!soloMax && valores[i] < calMin[i]) calMin[i] = valores[i];
    if (valores[i] > calMax[i])             calMax[i] = valores[i];
  }
}

int normalizar(int valor, int sensor) {
  if (calMax[sensor] == calMin[sensor]) return 50;
  return constrain(map(valor, calMin[sensor], calMax[sensor], 0, 100), 0, 100);
}


int calcularError(int norm[]) {
  const int pesos[6] = { -5, -3, -1, 1, 3, 5 };

  bool ladoIzq = (norm[0] > UMBRAL_NEGRO || norm[1] > UMBRAL_NEGRO);
  bool ladoDer = (norm[4] > UMBRAL_NEGRO || norm[5] > UMBRAL_NEGRO);
  if (ladoIzq && ladoDer) {
    LOGLN(">> BIFURCACION: tomando derecha");
    return 3;
  }

  long suma = 0, pesoTotal = 0;
  for (int i = 0; i < NUM_SENSORS; i++) {
    if (norm[i] > UMBRAL_NEGRO) {
      suma      += (long)norm[i] * pesos[i];
      pesoTotal += norm[i];
    }
  }

  if (pesoTotal == 0) return 999;
  return (int)(suma / pesoTotal);
}

void seguirLinea(int error) {

  integrador += error;
  integrador  = constrain(integrador, -LIMITE_INTEGRADOR, LIMITE_INTEGRADOR);
  float termI = Ki * integrador;

  // CORRECCIÓN TOTAL
  int correccion = (int)(termP + termD + termI);

  int velDer = constrain(VEL_BASE - correccion, -VEL_MAXIMA, VEL_MAXIMA);
  int velIzq = constrain(VEL_BASE + correccion, -VEL_MAXIMA, VEL_MAXIMA);

  motorDer(velDer);
  motorIzq(velIzq);
}

void manejarLineaPerdida() {
  if (!buscandoLinea) {
    buscandoLinea   = true;
    errorAlPerder   = ultimoError;
    ladoAlPerder    = ultimoLado;
    tInicioBusqueda = millis();

    integrador    = 0;
    errorAnterior = 0;

    LOG("!! LINEA PERDIDA !! Lado: "); LOG(ladoAlPerder);
    LOG(" Error: "); LOGLN(errorAlPerder);
  }

  if (millis() - tInicioBusqueda > T_MAX_BUSQUEDA) {
    LOGLN("!! BUSQUEDA AGOTADA: deteniendo robot.");
    detener();
    return;
  }

  int dir = (ladoAlPerder != 0) ? ladoAlPerder
          : (errorAlPerder > 0 ? 1 : -1);

  if (dir > 0) { motorDer(-VEL_BUSQUEDA); motorIzq( VEL_BUSQUEDA); }
  else         { motorDer( VEL_BUSQUEDA); motorIzq(-VEL_BUSQUEDA); }
}

void setup() {
  Serial.begin(9600);
  BT.begin(9600);

  pinMode(PIN_IN1, OUTPUT); pinMode(PIN_E1, OUTPUT);
  pinMode(PIN_IN2, OUTPUT); pinMode(PIN_E2, OUTPUT);

  detener();
  inicializarCalibracion();

  LOGLN("Calibrando...");
  for (int k = 0; k < LECTURAS_INIT; k++) {
    int raw[NUM_SENSORS];
    for (int i = 0; i < NUM_SENSORS; i++) raw[i] = analogRead(sensorPins[i]);
    actualizarCalibracion(raw, false);
    delay(5);
  }

  calibracionLista = true;
  LOGLN("Calibracion lista. Arrancando...");
}

void loop() {
  // 1. Leer
  int raw[NUM_SENSORS];
  for (int i = 0; i < NUM_SENSORS; i++) raw[i] = analogRead(sensorPins[i]);

  actualizarCalibracion(raw, calibracionLista);

  int norm[NUM_SENSORS];
  for (int i = 0; i < NUM_SENSORS; i++) norm[i] = normalizar(raw[i], i);

  int error = calcularError(norm);

  if (error == 999) {
    manejarLineaPerdida();
  } else {
    bool todoNegro = true;
    for (int i = 0; i < NUM_SENSORS; i++) {
      if (norm[i] <= UMBRAL_NEGRO) { todoNegro = false; break; }
    }

    if (buscandoLinea && todoNegro) {
      manejarLineaPerdida();
    } else {
      if (buscandoLinea) {
        LOGLN(">> Linea recuperada.");
        buscandoLinea = false;
        errorAnterior = error; // evita pico derivativo al recuperar
      }
      if      (norm[4] > UMBRAL_NEGRO || norm[5] > UMBRAL_NEGRO) ultimoLado =  1;
      else if (norm[0] > UMBRAL_NEGRO || norm[1] > UMBRAL_NEGRO) ultimoLado = -1;
      ultimoError = error;
      seguirLinea(error);
    }
  }

  // 6. Log — muestra los tres términos PID por separado
  //    para que puedas ver la contribución de cada uno
  LOG("NORM: ");
  for (int i = 0; i < NUM_SENSORS; i++) { LOG(norm[i]); LOG(" "); }
  LOG("| ERR: ");   LOG(error);
  LOG(" | P: ");    LOG((int)(Kp * error));
  LOG(" | D: ");    LOG((int)(Kd * (error - errorAnterior)));
  LOG(" | I: ");    LOG((int)(Ki * integrador));
  LOG(" | D/I: ");
  int corr = (int)(Kp*error + Kd*(error-errorAnterior) + Ki*integrador);
  LOG(constrain(VEL_BASE - corr, -VEL_MAXIMA, VEL_MAXIMA));
  LOG("/");
  LOGLN(constrain(VEL_BASE + corr, -VEL_MAXIMA, VEL_MAXIMA));

  delay(10);
}
