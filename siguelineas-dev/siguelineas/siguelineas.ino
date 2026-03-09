// ============================================================
//  SIGUELINEAS PID — ROBOT 3 RUEDAS
//  2 ruedas motrices delanteras + 1 rueda loca trasera
//  6 sensores IR delanteros (A0..A5)
//
//  CORRECCIÓN APLICADA:
//  - seguirLinea() ya no permite velocidades negativas (marcha atrás)
//    → los motores se limitan a [0, VEL_MAXIMA] durante el seguimiento
//  - Kd bajado de 16 a 10 para reducir picos en esquinas afiladas (rombos)
//
//  GUÍA DE AJUSTE PASO A PASO:
//
//  ── PASO 1: Solo P ──────────────────────────────────────────
//  Deja Kd=0 y Ki=0. Sube Kp de 5 en 5 hasta que:
//    ✓ Sigue curvas sin salirse
//    ✗ Pero zigzaguea en recta  ← normal, lo arregla el paso 2
//
//  ── PASO 2: Añadir D ────────────────────────────────────────
//  Con el Kp del paso 1, sube Kd de 2 en 2 hasta que:
//    ✓ El zigzag desaparece o se reduce mucho
//    ✗ Si Kd es muy alto: va rígido y no coge bien las curvas
//
//  ── PASO 3: Añadir I (solo si hace falta) ───────────────────
//  El I solo es necesario si el robot va desviado en rectas largas.
//  Sube Ki muy despacio (0.5 en 0.5) hasta que la deriva desaparezca.
//    ✗ Si Ki es muy alto: oscila de forma lenta y creciente
//
//  VALORES DE PARTIDA RECOMENDADOS:
//    Kp = 20, Kd = 0,  Ki = 0   ← empieza aquí (paso 1)
//    Kp = 20, Kd = 6,  Ki = 0   ← ejemplo paso 2
//    Kp = 20, Kd = 6,  Ki = 0.5 ← ejemplo paso 3
// ============================================================

#include <SoftwareSerial.h>
SoftwareSerial BT(10, 11);

#define LOG(x)   { Serial.print(x);   BT.print(x);   }
#define LOGLN(x) { Serial.println(x); BT.println(x); }

// ── SENSORES ─────────────────────────────────────────────────
const int sensorPins[6] = { A0, A1, A2, A3, A4, A5 };
const int NUM_SENSORS   = 6;

// ── MOTORES (L298N) ───────────────────────────────────────────
const int PIN_IN1 = 2;
const int PIN_E1  = 3;
const int PIN_IN2 = 4;
const int PIN_E2  = 5;

// ── VELOCIDADES ───────────────────────────────────────────────
const int VEL_BASE     = 240;
const int VEL_MAXIMA   = 255;
const int VEL_BUSQUEDA = 240;

// ── UMBRAL DE NEGRO (0-100 normalizado) ──────────────────────
const int UMBRAL_NEGRO = 65;

// ── CALIBRACIÓN ───────────────────────────────────────────────
int  calMin[NUM_SENSORS];
int  calMax[NUM_SENSORS];
const int LECTURAS_INIT = 200;
bool calibracionLista   = false;

// ============================================================
//  CONSTANTES PID  ←  AQUÍ AJUSTAS TÚ
//
//  PASO 1 — empieza así:
//    Kp = 20.0 | Kd = 0.0 | Ki = 0.0
//
//  PASO 2 — cuando P ya funciona, activa D:
//    Kp = 20.0 | Kd = 6.0 | Ki = 0.0
//
//  PASO 3 — solo si hay deriva en recta, activa I:
//    Kp = 20.0 | Kd = 6.0 | Ki = 0.5
//
//  NOTA: Kd bajado de 16 → 10 para evitar picos en esquinas de rombos
// ============================================================
const float Kp = 10.0;   // ← PASO 1: sube de 5 en 5
const float Kd = 0.0;   // ← PASO 2: sube de 2 en 2 
const float Ki =  0.0;   // ← PASO 3: sube de 0.5 en 0.5

// ── VARIABLES INTERNAS DEL PID ───────────────────────────────
int   errorAnterior  = 0;
float integrador     = 0.0;

// Límite del integrador: evita que Ki acumule demasiado y
// descontrole el robot (fenómeno llamado "integrator windup").
// Si notas que el robot tarda mucho en corregir tras una curva,
// baja este valor. Si la deriva no se corrige del todo, súbelo.
const float LIMITE_INTEGRADOR = 100.0;

// ── ESTADO DE BÚSQUEDA ────────────────────────────────────────
int  ultimoError      = 0;
int  ultimoLado       = 1;
int  ladoAlPerder     = 1;
int  errorAlPerder    = 0;
bool buscandoLinea    = false;
unsigned long tInicioBusqueda = 0;
const unsigned long T_MAX_BUSQUEDA = 9000;

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
//  CALIBRACIÓN
// ============================================================

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

// ============================================================
//  CÁLCULO DEL ERROR
//  Pesos: S0(izq extremo)=-5 ... S5(der extremo)=+5
//  Negativo → línea a la izquierda
//  Positivo → línea a la derecha
//  999      → línea perdida
// ============================================================

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

// ============================================================
//  CONTROL PID
//
//  corrección = Kp·error  +  Kd·(error-errorAnterior)  +  Ki·∑error
//               ─────────    ──────────────────────────    ─────────
//               PASO 1            PASO 2                   PASO 3
//
//  La corrección se resta al motor derecho y se suma al izquierdo.
//  Así, si la línea está a la derecha (error>0) el robot gira derecha.
//
//  *** CORRECCIÓN MARCHA ATRÁS ***
//  Los motores se limitan a [0, VEL_MAXIMA] en lugar de [-VEL_MAXIMA, VEL_MAXIMA].
//  En las esquinas afiladas de los rombos, la corrección puede superar 500,
//  lo que antes invertía los motores. Ahora el motor más lento se detiene
//  (pivote) en lugar de girar al revés.
// ============================================================

void seguirLinea(int error) {

  // ── TÉRMINO P ── (PASO 1) ────────────────────────────────
  float termP = Kp * error;

  // ── TÉRMINO D ── (PASO 2, Kd > 0) ───────────────────────
  float termD = Kd * (error - errorAnterior);
  errorAnterior = error;

  // ── TÉRMINO I ── (PASO 3, Ki > 0) ───────────────────────
  integrador += error;
  integrador  = constrain(integrador, -LIMITE_INTEGRADOR, LIMITE_INTEGRADOR);
  float termI = Ki * integrador;

  // ── CORRECCIÓN TOTAL ─────────────────────────────────────
  int correccion = (int)(termP + termD + termI);

  // *** CAMBIO CLAVE: límite inferior 0 en lugar de -VEL_MAXIMA ***
  // Esto impide que los motores vayan marcha atrás durante el seguimiento,
  // evitando el comportamiento erróneo en los vértices de los rombos.
  int velDer = constrain(VEL_BASE - correccion, 0, VEL_MAXIMA);
  int velIzq = constrain(VEL_BASE + correccion, 0, VEL_MAXIMA);

  motorDer(velDer);
  motorIzq(velIzq);
}

// ============================================================
//  BÚSQUEDA DE LÍNEA
// ============================================================

void manejarLineaPerdida() {
  if (!buscandoLinea) {
    buscandoLinea   = true;
    errorAlPerder   = ultimoError;
    ladoAlPerder    = ultimoLado;
    tInicioBusqueda = millis();

    // Resetear integrador: al perder la línea el acumulado
    // ya no es válido y podría desorientar al recuperarla.
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

  // Giro en sitio hacia el último lado donde se vio la línea
  // (aquí sí se permiten velocidades negativas: es búsqueda activa)
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

// ============================================================
//  LOOP
// ============================================================

void loop() {
  // 1. Leer
  int raw[NUM_SENSORS];
  for (int i = 0; i < NUM_SENSORS; i++) raw[i] = analogRead(sensorPins[i]);

  // 2. Calibración dinámica
  actualizarCalibracion(raw, calibracionLista);

  // 3. Normalizar
  int norm[NUM_SENSORS];
  for (int i = 0; i < NUM_SENSORS; i++) norm[i] = normalizar(raw[i], i);

  // 4. Error
  int error = calcularError(norm);

  // 5. Actuar
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
        errorAnterior = error;
      }
      if      (norm[4] > UMBRAL_NEGRO || norm[5] > UMBRAL_NEGRO) ultimoLado =  1;
      else if (norm[0] > UMBRAL_NEGRO || norm[1] > UMBRAL_NEGRO) ultimoLado = -1;
      ultimoError = error;
      seguirLinea(error);
    }
  }

  // 6. Log — muestra los tres términos PID por separado
  LOG("NORM: ");
  for (int i = 0; i < NUM_SENSORS; i++) { LOG(norm[i]); LOG(" "); }
  LOG("| ERR: ");   LOG(error);
  LOG(" | P: ");    LOG((int)(Kp * error));
  LOG(" | D: ");    LOG((int)(Kd * (error - errorAnterior)));
  LOG(" | I: ");    LOG((int)(Ki * integrador));
  LOG(" | D/I: ");
  int corr = (int)(Kp*error + Kd*(error-errorAnterior) + Ki*integrador);
  LOG(constrain(VEL_BASE - corr, 0, VEL_MAXIMA));
  LOG("/");
  LOGLN(constrain(VEL_BASE + corr, 0, VEL_MAXIMA));

  delay(10);
}
