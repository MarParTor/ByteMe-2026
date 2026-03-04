// ============================================================
//  SIGUELINEAS — ROBOT 3 RUEDAS
//  2 ruedas motrices delanteras (izq/der) + 1 rueda loca trasera
//  6 sensores IR en la parte delantera (A0..A5)
//
//  Convención sensores:
//    valor ALTO  = negro  (poca reflexión)
//    valor BAJO  = blanco (mucha reflexión)
//
//  Control: proporcional-derivativo (PD)
//  Motores: L298N igual que el código de referencia
// ============================================================

#include <SoftwareSerial.h>
SoftwareSerial BT(10, 11); // RX, TX

#define LOG(x)   { Serial.print(x);   BT.print(x);   }
#define LOGLN(x) { Serial.println(x); BT.println(x); }

// ── SENSORES ─────────────────────────────────────────────────
const int sensorPins[6] = { A0, A1, A2, A3, A4, A5 };
const int NUM_SENSORS   = 6;

// ── MOTORES (L298N) ───────────────────────────────────────────
//   Motor derecho  → IN1 / E1
//   Motor izquierdo → IN2 / E2
const int PIN_IN1 = 2;
const int PIN_E1  = 3;
const int PIN_IN2 = 4;
const int PIN_E2  = 5;

// ── VELOCIDADES ───────────────────────────────────────────────
const int VEL_BASE     = 180;   // velocidad de crucero
const int VEL_MAXIMA   = 255;
const int VEL_BUSQUEDA = 160;   // velocidad al girar buscando línea

// ── UMBRAL DE DETECCIÓN DE NEGRO (0-100 normalizado) ─────────
const int UMBRAL_NEGRO = 50;

// ── CALIBRACIÓN AUTOMÁTICA ────────────────────────────────────
int  calMin[NUM_SENSORS];
int  calMax[NUM_SENSORS];
const int LECTURAS_INIT = 200;  // muestras durante el arranque
bool calibracionLista   = false;

// ── CONTROL PD ───────────────────────────────────────────────
//   Kp: cuánto corregimos en proporción al error
//   Kd: amortigua oscilaciones (derivativo)
const float Kp = 10.0;
//const float Kd =  8.0;
int errorAnterior = 0;

// ── ESTADO DE BÚSQUEDA ────────────────────────────────────────
int  ultimoError      = 0;
int  ultimoLado       = 1;   // +1 derecha, -1 izquierda
int  ladoAlPerder     = 1;
int  errorAlPerder    = 0;
bool buscandoLinea    = false;
unsigned long tInicioBusqueda = 0;
const unsigned long T_MAX_BUSQUEDA = 3000; // ms antes de detenerse

// ============================================================
//  MOTORES
// ============================================================

// vel > 0 → adelante  |  vel < 0 → atrás  |  vel = 0 → freno
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
//  CALIBRACIÓN
// ============================================================

void inicializarCalibracion() {
  for (int i = 0; i < NUM_SENSORS; i++) {
    calMin[i] = 1023;
    calMax[i] = 0;
  }
}

// soloMax = true  → solo amplía el máximo (durante seguimiento)
// soloMax = false → amplía ambos extremos  (durante arranque)
void actualizarCalibracion(int valores[], bool soloMax) {
  for (int i = 0; i < NUM_SENSORS; i++) {
    if (!soloMax && valores[i] < calMin[i]) calMin[i] = valores[i];
    if (valores[i] > calMax[i])             calMax[i] = valores[i];
  }
}

// Devuelve 0-100: 0 = blanco puro, 100 = negro puro
int normalizar(int valor, int sensor) {
  if (calMax[sensor] == calMin[sensor]) return 50;
  return constrain(map(valor, calMin[sensor], calMax[sensor], 0, 100), 0, 100);
}

// ============================================================
//  CÁLCULO DEL ERROR
//  Pesos: S0(izq extremo) = -5 ... S5(der extremo) = +5
//  Error negativo → línea a la izquierda → hay que girar izq
//  Error positivo → línea a la derecha   → hay que girar der
//  Error = 999    → ningún sensor ve negro (línea perdida)
// ============================================================

int calcularError(int norm[]) {
  const int pesos[6] = { -5, -3, -1, 1, 3, 5 };

  // Bifurcación: sensores extremos de ambos lados activos
  bool ladoIzq = (norm[0] > UMBRAL_NEGRO || norm[1] > UMBRAL_NEGRO);
  bool ladoDer = (norm[4] > UMBRAL_NEGRO || norm[5] > UMBRAL_NEGRO);
  if (ladoIzq && ladoDer) {
    LOGLN(">> BIFURCACION: tomando derecha");
    return 3; // pequeña corrección a la derecha
  }

  long suma = 0, pesoTotal = 0;
  for (int i = 0; i < NUM_SENSORS; i++) {
    if (norm[i] > UMBRAL_NEGRO) {
      suma      += (long)norm[i] * pesos[i];
      pesoTotal += norm[i];
    }
  }

  if (pesoTotal == 0) return 999; // línea perdida

  return (int)(suma / pesoTotal);
}

// ============================================================
//  SEGUIR LÍNEA  —  control PD
// ============================================================

void seguirLinea(int error) {
  int derivativo = error - errorAnterior;
  errorAnterior  = error;

  //int correccion = (int)(Kp * error + Kd * derivativo);
    //int correccion = (int)(Kp * error + * derivativo);

  // Motor que va por el lado exterior de la curva recibe menos
  // velocidad; el interior puede llegar a ir marcha atrás en
  // curvas muy cerradas (correccion grande).
  int velDer = constrain(VEL_BASE - correccion, -VEL_MAXIMA, VEL_MAXIMA);
  int velIzq = constrain(VEL_BASE + correccion, -VEL_MAXIMA, VEL_MAXIMA);

  motorDer(velDer);
  motorIzq(velIzq);
}

// ============================================================
//  BÚSQUEDA DE LÍNEA
//  Gira en el último lado donde estaba la línea.
//  Si supera T_MAX_BUSQUEDA, se detiene.
// ============================================================

void manejarLineaPerdida() {
  if (!buscandoLinea) {
    buscandoLinea   = true;
    errorAlPerder   = ultimoError;
    ladoAlPerder    = ultimoLado;
    tInicioBusqueda = millis();
    LOG("!! LINEA PERDIDA !! Lado: "); LOG(ladoAlPerder);
    LOG(" Error: "); LOGLN(errorAlPerder);
  }

  if (millis() - tInicioBusqueda > T_MAX_BUSQUEDA) {
    LOGLN("!! BUSQUEDA AGOTADA: deteniendo robot.");
    detener();
    return;
  }

  // Dirección de giro: hacia donde se fue la línea
  int dir = (ladoAlPerder != 0) ? ladoAlPerder
          : (errorAlPerder > 0 ? 1 : -1);

  // Giro en sitio: una rueda adelante, la otra atrás
  if (dir > 0) { motorDer(-VEL_BUSQUEDA); motorIzq( VEL_BUSQUEDA); }
  else         { motorDer( VEL_BUSQUEDA); motorIzq(-VEL_BUSQUEDA); }
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
  inicializarCalibracion();

  // Calibración inicial: el robot está parado sobre la pista.
  // Los sensores ven blanco y negro → establece los extremos.
  LOGLN("Calibrando... coloca el robot sobre la pista.");
  for (int k = 0; k < LECTURAS_INIT; k++) {
    int raw[NUM_SENSORS];
    for (int i = 0; i < NUM_SENSORS; i++) raw[i] = analogRead(sensorPins[i]);
    actualizarCalibracion(raw, false);
    delay(5);
  }

  calibracionLista = true;
  LOGLN("Calibracion lista. Arrancando...");
}

// ============================================================
//  LOOP
// ============================================================

void loop() {
  // 1. Leer sensores
  int raw[NUM_SENSORS];
  for (int i = 0; i < NUM_SENSORS; i++) raw[i] = analogRead(sensorPins[i]);

  // 2. Actualizar calibración dinámica (solo max durante marcha)
  actualizarCalibracion(raw, calibracionLista);

  // 3. Normalizar a 0-100
  int norm[NUM_SENSORS];
  for (int i = 0; i < NUM_SENSORS; i++) norm[i] = normalizar(raw[i], i);

  // 4. Calcular error de posición
  int error = calcularError(norm);

  // 5. Actuar según el error
  if (error == 999) {
    // Ningún sensor ve negro
    manejarLineaPerdida();
  } else {
    // Comprobar "todo negro" (cruce de línea gruesa o interferencia)
    bool todoNegro = true;
    for (int i = 0; i < NUM_SENSORS; i++) {
      if (norm[i] <= UMBRAL_NEGRO) { todoNegro = false; break; }
    }

    if (buscandoLinea && todoNegro) {
      // Seguimos buscando, esto no es línea real
      manejarLineaPerdida();
    } else {
      if (buscandoLinea) {
        LOGLN(">> Linea recuperada.");
        buscandoLinea = false;
        errorAnterior = error; // evita pico derivativo al recuperar
      }

      // Actualizar último lado donde se vio la línea
      if      (norm[4] > UMBRAL_NEGRO || norm[5] > UMBRAL_NEGRO) ultimoLado =  1;
      else if (norm[0] > UMBRAL_NEGRO || norm[1] > UMBRAL_NEGRO) ultimoLado = -1;

      ultimoError = error;
      seguirLinea(error);
    }
  }

  // 6. Log de depuración por Serial y BT
  LOG("RAW: ");
  for (int i = 0; i < NUM_SENSORS; i++) { LOG(raw[i]); LOG(" "); }
  LOG("| NORM: ");
  for (int i = 0; i < NUM_SENSORS; i++) { LOG(norm[i]); LOG(" "); }
  LOG("| ERR: "); LOG(error);
  if (error != 999) {
    int corr = (int)(Kp * error + Kd * (error - errorAnterior));
    LOG(" | D/I: ");
    LOG(constrain(VEL_BASE - corr, -VEL_MAXIMA, VEL_MAXIMA));
    LOG("/");
    LOGLN(constrain(VEL_BASE + corr, -VEL_MAXIMA, VEL_MAXIMA));
  } else {
    LOG(" | Buscando t=");
    LOGLN(buscandoLinea ? (int)(millis() - tInicioBusqueda) : 0);
  }

  delay(10);
}