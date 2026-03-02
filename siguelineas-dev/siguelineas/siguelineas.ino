// ============================================================
//  SIGUELINEAS CON AUTOCALIBRACIÓN DINÁMICA
//  6 sensores IR reflectivos (OUT0..OUT5 en A0..A5)
//  Convención del hardware:
//    1023 = negro más puro   (mucha reflexión IR absorbida)
//       0 = blanco más puro  (mucha reflexión IR reflejada)
//  IMPORTANTE: Los colores como rojo o azul quedan en valores
//  intermedios según su reflectividad IR. La autocalibración
//  dinámica se adapta automáticamente a estos colores.
// ============================================================

// ── BLUETOOTH ────────────────────────────────────────────────
#include <SoftwareSerial.h>
SoftwareSerial BT(10, 11); // RX, TX

#define LOG(x)   { Serial.print(x);   BT.print(x);   }
#define LOGLN(x) { Serial.println(x); BT.println(x); }

// ── PINES SENSORES ───────────────────────────────────────────
const int sensorPins[6] = { A0, A1, A2, A3, A4, A5 };
const int NUM_SENSORS   = 6;

// ── PINES DE MOTORES (L298N) ─────────────────────────────────
const int PIN_IN1 = 2;
const int PIN_E1  = 3;
const int PIN_IN2 = 4;
const int PIN_E2  = 5;

// ── PARÁMETROS DE VELOCIDAD ──────────────────────────────────
const int VEL_BASE    = 220; // antes 190
const int VEL_MAXIMA  = 255;
const int VEL_BUSQUEDA = 220; // antes 210 // antes 190

// ── AUTOCALIBRACIÓN ──────────────────────────────────────────
int calMin[NUM_SENSORS];
int calMax[NUM_SENSORS];

const int LECTURAS_INIT = 200;

// ── CONTROL DE CALIBRACIÓN ───────────────────────────────────
// FIX PRINCIPAL: una vez que la calibración inicial está lista,
// solo actualizamos calMax (podemos ver negros más negros si
// el robot pasa por colores oscuros nuevos), pero NUNCA
// actualizamos calMin durante el seguimiento.
// Así evitamos que un "todo blanco" corrompa la referencia de blanco.
bool calibracionLista = false;

// ── UMBRAL NORMALIZADO ───────────────────────────────────────
const int UMBRAL_NEGRO = 50;

// ── ESTADO DE BÚSQUEDA ───────────────────────────────────────
int  ultimoError     = 0;
bool buscandoLinea   = false;
unsigned long tInicioBusqueda = 0;

// Tiempo máximo de búsqueda antes de detenerse (ms)
// Si en este tiempo no encuentra la línea, para y espera.
const unsigned long T_MAX_BUSQUEDA = 3000;

// ============================================================
//  FUNCIONES DE MOTORES
// ============================================================

void motorDer(int vel) {
  vel = constrain(vel, -255, 255);
  if (vel > 0) {
    digitalWrite(PIN_IN1, HIGH);
    analogWrite(PIN_E1, vel);
  } else if (vel < 0) {
    digitalWrite(PIN_IN1, LOW);
    analogWrite(PIN_E1, -vel);
  } else {
    digitalWrite(PIN_IN1, HIGH);
    analogWrite(PIN_E1, 0);
  }
}

void motorIzq(int vel) {
  vel = constrain(vel, -255, 255);
  if (vel > 0) {
    digitalWrite(PIN_IN2, HIGH);
    analogWrite(PIN_E2, vel);
  } else if (vel < 0) {
    digitalWrite(PIN_IN2, LOW);
    analogWrite(PIN_E2, -vel);
  } else {
    digitalWrite(PIN_IN2, HIGH);
    analogWrite(PIN_E2, 0);
  }
}

void detener() {
  analogWrite(PIN_E1, 0);
  analogWrite(PIN_E2, 0);
}

// ============================================================
//  AUTOCALIBRACIÓN
// ============================================================

void inicializarCalibracion() {
  for (int i = 0; i < NUM_SENSORS; i++) {
    calMin[i] = 1023;
    calMax[i] = 0;
  }
}

// ── FIX: dos modos de actualización ─────────────────────────
// Durante la fase inicial → actualizamos AMBOS extremos (aprendemos todo)
// Durante el seguimiento  → actualizamos SOLO calMax
//   Razón: si el robot ve todo blanco (línea perdida), no queremos
//   que ese blanco "desplace" el calMin y corrompa la normalización.
//   Los negros más negros sí los aceptamos (adaptación a colores oscuros).
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

// ============================================================
//  LÓGICA DE SEGUIMIENTO DE LÍNEA
// ============================================================

int calcularError(int norm[]) {
  const int pesos[6] = { -5, -3, -1, 1, 3, 5 };

  bool ladoIzq = (norm[0] > UMBRAL_NEGRO || norm[1] > UMBRAL_NEGRO);
  bool ladoDer = (norm[4] > UMBRAL_NEGRO || norm[5] > UMBRAL_NEGRO);
  if (ladoIzq && ladoDer) {
    LOG(">> BIFURCACION: tomando derecha");
    LOGLN("");
    return 3;
  }

  long suma      = 0;
  long pesoTotal = 0;

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
  const float Kp = 20.0;

  int correccion = (int)(Kp * error);

  int velDer = constrain(VEL_BASE - correccion, -VEL_MAXIMA, VEL_MAXIMA);
  int velIzq = constrain(VEL_BASE + correccion, -VEL_MAXIMA, VEL_MAXIMA);

  motorDer(velDer);
  motorIzq(velIzq);
}

// ── FIX: búsqueda con tiempo límite ──────────────────────────
// Al perder la línea:
//   - Arranca el temporizador la primera vez
//   - Gira hacia el lado donde se perdió la línea
//   - Si supera T_MAX_BUSQUEDA ms sin encontrarla → para el robot
//     (evita que se aleje indefinidamente)
void manejarLineaPerdida() {
  if (!buscandoLinea) {
    // Primera iteración sin línea: arrancar temporizador
    buscandoLinea    = true;
    tInicioBusqueda  = millis();
    LOG("!! LINEA PERDIDA !! Girando hacia: ");
    LOGLN(ultimoError);
  }

  // Comprobar si se ha superado el tiempo máximo de búsqueda
  if (millis() - tInicioBusqueda > T_MAX_BUSQUEDA) {
    LOGLN("!! BUSQUEDA AGOTADA: deteniendo robot.");
    detener();
    return;
  }

  // Girar hacia el lado donde se vio la línea por última vez
  if (ultimoError > 0) {
    motorDer(-VEL_BUSQUEDA);
    motorIzq( VEL_BUSQUEDA);
  } else if (ultimoError < 0) {
    motorDer( VEL_BUSQUEDA);
    motorIzq(-VEL_BUSQUEDA);
  } else {
    // Sin historial: gira a la derecha por defecto
    motorDer(-VEL_BUSQUEDA);
    motorIzq( VEL_BUSQUEDA);
  }
}

// ============================================================
//  SETUP
// ============================================================
void setup() {
  Serial.begin(9600);
  BT.begin(9600);

  pinMode(PIN_IN1, OUTPUT);
  pinMode(PIN_E1,  OUTPUT);
  pinMode(PIN_IN2, OUTPUT);
  pinMode(PIN_E2,  OUTPUT);

  detener();
  inicializarCalibracion();

  LOGLN("Calibrando...");
  for (int k = 0; k < LECTURAS_INIT; k++) {
    int raw[NUM_SENSORS];
    for (int i = 0; i < NUM_SENSORS; i++) raw[i] = analogRead(sensorPins[i]);
    // En la fase inicial actualizamos AMBOS extremos
    actualizarCalibracion(raw, false);
    delay(5);
  }

  calibracionLista = true;
  LOGLN("Calibracion inicial lista. Iniciando...");
}

// ============================================================
//  LOOP PRINCIPAL
// ============================================================
void loop() {
  // 1. Leer sensores
  int raw[NUM_SENSORS];
  for (int i = 0; i < NUM_SENSORS; i++) raw[i] = analogRead(sensorPins[i]);

  // 2. Autocalibración: durante el seguimiento solo ampliamos calMax
  //    para no corromper el blanco de referencia.
  actualizarCalibracion(raw, calibracionLista);

  // 3. Normalizar
  int norm[NUM_SENSORS];
  for (int i = 0; i < NUM_SENSORS; i++) norm[i] = normalizar(raw[i], i);

  // 4. Calcular error
  int error = calcularError(norm);

  // 5. Actuar
  if (error == 999) {
    manejarLineaPerdida();
  } else {
    // Línea encontrada: resetear estado de búsqueda
    if (buscandoLinea) {
      LOGLN(">> Linea recuperada.");
      buscandoLinea = false;
    }
    ultimoError = error;
    seguirLinea(error);
  }

  // 6. Chivatos
  LOG("Norm: ");
  for (int i = 0; i < NUM_SENSORS; i++) { LOG(norm[i]); LOG(" "); }
  LOG("| Error: "); LOG(error);
  if (error != 999) {
    LOG(" | Vel D/I: ");
    LOG(constrain(VEL_BASE - (int)(20.0 * error), -VEL_MAXIMA, VEL_MAXIMA));
    LOG("/");
    LOGLN(constrain(VEL_BASE + (int)(20.0 * error), -VEL_MAXIMA, VEL_MAXIMA));
  } else {
    LOG(" | Buscando... t=");
    LOGLN(buscandoLinea ? (int)(millis() - tInicioBusqueda) : 0);
  }

  delay(10);
}
