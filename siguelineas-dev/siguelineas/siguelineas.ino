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
// RX del Arduino al TX del HC-05, TX del Arduino al RX del HC-05
// Pines 10 y 11 libres para el bluetooth
#include <SoftwareSerial.h>
SoftwareSerial BT(10, 11); // RX, TX

// Macro para enviar los chivatos a los dos puertos a la vez (Serie y Bluetooth)
#define LOG(x)   { Serial.print(x);   BT.print(x);   }
#define LOGLN(x) { Serial.println(x); BT.println(x); }

// ── PINES SENSORES ───────────────────────────────────────────
const int sensorPins[6] = { A0, A1, A2, A3, A4, A5 };
const int NUM_SENSORS   = 6;

// ── PINES DE MOTORES (L298N) ─────────────────────────────────
// M1 = motor derecha:  IN1 dirección, E1 velocidad PWM
// M2 = motor izquierda: IN2 dirección, E2 velocidad PWM
const int PIN_IN1 = 2;  // Dirección motor derecha
const int PIN_E1  = 3;  // Velocidad motor derecha (PWM)
const int PIN_IN2 = 4;  // Dirección motor izquierda
const int PIN_E2  = 5;  // Velocidad motor izquierda (PWM)

// ── PARÁMETROS DE VELOCIDAD ──────────────────────────────────
const int VEL_BASE   = 160;  // 0-255: velocidad crucero
const int VEL_MAXIMA = 255;  // velocidad máxima en giros

// ── AUTOCALIBRACIÓN ──────────────────────────────────────────
// Se almacena el mínimo (más blanco) y máximo (más negro) visto
// en tiempo real para cada sensor.
int calMin[NUM_SENSORS];   // valor más bajo visto  → blanco
int calMax[NUM_SENSORS];   // valor más alto visto  → negro

// Cuántas lecturas iniciales forzamos para "sembrar" la calibración
// antes de empezar a mover el robot.
const int LECTURAS_INIT = 200;

// ── UMBRAL NORMALIZADO ───────────────────────────────────────
// Tras normalizar 0-100, un valor mayor que este se considera NEGRO.
// Funciona también para colores oscuros como rojo oscuro o azul marino.
const int UMBRAL_NEGRO = 50;


// Último error válido antes de perder la línea
int ultimoError = 0;
// ============================================================
//  FUNCIONES DE MOTORES
// ============================================================

/**
 * motorDer(vel)
 * Controla el motor derecho (M1).
 * vel > 0 → adelante, vel < 0 → atrás (para giros cerrados), vel = 0 → freno.
 */
void motorDer(int vel) {
  vel = constrain(vel, -255, 255);
  if (vel > 0) {
    digitalWrite(PIN_IN1, HIGH);
    analogWrite(PIN_E1, vel);
  } else if (vel < 0) {
    // Marcha atrás: permite giros sobre sí mismo con más fuerza
    digitalWrite(PIN_IN1, LOW);
    analogWrite(PIN_E1, -vel);
  } else {
    digitalWrite(PIN_IN1, HIGH);
    analogWrite(PIN_E1, 0);
  }
}

/**
 * motorIzq(vel)
 * Controla el motor izquierdo (M2).
 * vel > 0 → adelante, vel < 0 → atrás (para giros cerrados), vel = 0 → freno.
 */
void motorIzq(int vel) {
  vel = constrain(vel, -255, 255);
  if (vel > 0) {
    digitalWrite(PIN_IN2, HIGH);
    analogWrite(PIN_E2, vel);
  } else if (vel < 0) {
    // Marcha atrás: permite giros sobre sí mismo con más fuerza
    digitalWrite(PIN_IN2, LOW);
    analogWrite(PIN_E2, -vel);
  } else {
    digitalWrite(PIN_IN2, HIGH);
    analogWrite(PIN_E2, 0);
  }
}

/**
 * detener()
 * Para ambos motores inmediatamente.
 */
void detener() {
  analogWrite(PIN_E1, 0);
  analogWrite(PIN_E2, 0);
}

// ============================================================
//  AUTOCALIBRACIÓN
// ============================================================

/**
 * inicializarCalibracion()
 * Siembra los límites con valores extremos opuestos para que
 * la primera lectura real siempre los actualice.
 */
void inicializarCalibracion() {
  for (int i = 0; i < NUM_SENSORS; i++) {
    calMin[i] = 1023;  // empezamos asumiendo que todo es negro
    calMax[i] = 0;     // empezamos asumiendo que todo es blanco
  }
}

/**
 * actualizarCalibracion(valores[])
 * Recibe las lecturas crudas del ADC y actualiza los extremos
 * dinámicamente. Se llama en cada iteración del loop.
 *
 * De este modo, si el robot pasa por zonas con colores como rojo
 * o azul, los límites se amplían automáticamente sin reiniciar.
 */
void actualizarCalibracion(int valores[]) {
  for (int i = 0; i < NUM_SENSORS; i++) {
    if (valores[i] < calMin[i]) calMin[i] = valores[i];
    if (valores[i] > calMax[i]) calMax[i] = valores[i];
  }
}

/**
 * normalizar(valor, sensor)
 * Mapea la lectura cruda al rango 0-100 usando los extremos
 * calibrados de ese sensor:
 *   0   → blanco más puro visto (calMin)
 *   100 → negro más puro visto  (calMax)
 *
 * Si calMin == calMax (aún sin datos suficientes) devuelve 50
 * para evitar división por cero.
 */
int normalizar(int valor, int sensor) {
  if (calMax[sensor] == calMin[sensor]) return 50;
  return map(valor, calMin[sensor], calMax[sensor], 0, 100);
}

// ============================================================
//  LÓGICA DE SEGUIMIENTO DE LÍNEA
// ============================================================

/**
 * calcularError(normalizados[])
 * Calcula un error proporcional a la posición de la línea negra
 * respecto al centro del array de sensores.
 *
 * Esquema de pesos para 6 sensores (izq → der):
 *   S0=-5  S1=-3  S2=-1  S3=+1  S4=+3  S5=+5
 *
 * Si los sensores izquierdos ven negro  → error negativo → girar izquierda
 * Si los sensores derechos ven negro    → error positivo → girar derecha
 * Si no se detecta línea               → devuelve 999 (línea perdida)
 */
int calcularError(int norm[]) {
  const int pesos[6] = { -5, -3, -1, 1, 3, 5 };

  // ── DETECCIÓN DE BIFURCACIÓN ─────────────────────────────
  // Si los sensores extremos izquierdos Y derechos ven negro
  // simultáneamente, hay una bifurcación: tomamos la derecha.
  bool ladoIzq = (norm[0] > UMBRAL_NEGRO || norm[1] > UMBRAL_NEGRO);
  bool ladoDer = (norm[4] > UMBRAL_NEGRO || norm[5] > UMBRAL_NEGRO);
  if (ladoIzq && ladoDer) {
    LOG(">> BIFURCACION: tomando derecha");
    LOGLN("");
    return 3;  // Error positivo → el robot gira a la derecha
  }

  long suma      = 0;
  long pesoTotal = 0;

  for (int i = 0; i < NUM_SENSORS; i++) {
    if (norm[i] > UMBRAL_NEGRO) {
      suma      += (long)norm[i] * pesos[i];
      pesoTotal += norm[i];
    }
  }

  if (pesoTotal == 0) return 999;  // línea perdida

  // Error: -5 a +5, escalado a entero
  return (int)(suma / pesoTotal);
}

/**
 * seguirLinea(error)
 * Aplica una corrección proporcional simple (P) a los motores.
 * error < 0 → línea a la izquierda → el robot gira izquierda (motor izq más lento)
 * error > 0 → línea a la derecha   → el robot gira derecha (motor der más lento)
 *
 * La corrección se suma/resta a la velocidad base de cada motor.
 * Cuanto mayor |error|, más agresivo el giro.
 */
void seguirLinea(int error) {
  // Ganancia proporcional: ajusta según la pista real
  // Sube Kp si el robot reacciona poco, bájalo si va zigzagueando
  const float Kp = 20.0;

  int correccion = (int)(Kp * error);

  int velDer = VEL_BASE - correccion;
  int velIzq = VEL_BASE + correccion;

  // Permite negativos: el motor interno gira hacia atrás en curvas cerradas
  // dando mucha más fuerza al giro
  velDer = constrain(velDer, -VEL_MAXIMA, VEL_MAXIMA);
  velIzq = constrain(velIzq, -VEL_MAXIMA, VEL_MAXIMA);

  motorDer(velDer);
  motorIzq(velIzq);
}

/**
 * manejarLineaPerdida()
 * Estrategia cuando no se detecta línea: frena para no perderse más.
 */
void manejarLineaPerdida() {
  LOG("!! LINEA PERDIDA !! Girando hacia: ");
  LOGLN(ultimoError);

  // Velocidad de búsqueda: más lenta que la crucero para no pasarse
  const int VEL_BUSQUEDA = 140;

  if (ultimoError > 0) {
    // La línea se perdió por la derecha → gira derecha
    motorDer(-VEL_BUSQUEDA);
    motorIzq(VEL_BUSQUEDA);
  } else if (ultimoError < 0) {
    // La línea se perdió por la izquierda → gira izquierda
    motorDer(VEL_BUSQUEDA);
    motorIzq(-VEL_BUSQUEDA);
  } else {
    // Nunca hubo error conocido → frena (caso de arranque sin línea)
    detener();
  }
}

// ============================================================
//  SETUP
// ============================================================
void setup() {
  Serial.begin(9600);
  BT.begin(9600); // Asegúrate de que el HC-05 esté configurado a 9600

  // Configurar pines de motores como salida
  pinMode(PIN_IN1, OUTPUT);
  pinMode(PIN_E1,  OUTPUT);
  pinMode(PIN_IN2, OUTPUT);
  pinMode(PIN_E2,  OUTPUT);

  detener();
  inicializarCalibracion();

  // ── FASE DE CALIBRACIÓN INICIAL ──────────────────────────
  // Antes de moverse, el robot toma LECTURAS_INIT muestras
  // estático para establecer un rango base de calibración.
  // En este punto conviene mover el robot manualmente sobre
  // blanco y negro si es posible, pero si no, con quedarse
  // quieto ya siembra algo útil.
  LOGLN("Calibrando...");
  for (int k = 0; k < LECTURAS_INIT; k++) {
    int raw[NUM_SENSORS];
    for (int i = 0; i < NUM_SENSORS; i++) {
      raw[i] = analogRead(sensorPins[i]);
    }
    actualizarCalibracion(raw);
    delay(5);
  }
  LOGLN("Calibracion inicial lista. Iniciando...");
}

// ============================================================
//  LOOP PRINCIPAL
// ============================================================
void loop() {
  // 1. Leer sensores
  int raw[NUM_SENSORS];
  for (int i = 0; i < NUM_SENSORS; i++) {
    raw[i] = analogRead(sensorPins[i]);
  }

  // 2. Autocalibración continua: ampliar rango si encontramos
  //    valores más extremos que los vistos hasta ahora.
  //    Esto hace que el robot se adapte a los colores del circuito en marcha.
  actualizarCalibracion(raw);

  // 3. Normalizar cada sensor a 0-100
  int norm[NUM_SENSORS];
  for (int i = 0; i < NUM_SENSORS; i++) {
    norm[i] = normalizar(raw[i], i);
  }

  // 4. Calcular posición de la línea
  int error = calcularError(norm);

  // 5. Actuar según el error
  if (error == 999) {
    manejarLineaPerdida();
  } else {
      ultimoError = error;  // ← guarda el último error válido
    seguirLinea(error);
  }

  // 6. Chivatos por Bluetooth y Serie
  LOG("Norm: ");
  for (int i = 0; i < NUM_SENSORS; i++) {
    LOG(norm[i]); LOG(" ");
  }
  LOG("| Error: "); LOG(error);
  if (error != 999) {
    LOG(" | Vel D/I: ");
    LOG(constrain(VEL_BASE - (int)(20.0 * error), 0, VEL_MAXIMA));
    LOG("/");
    LOGLN(constrain(VEL_BASE + (int)(20.0 * error), 0, VEL_MAXIMA));
  } else {
    LOGLN("");
  }

  delay(10);  // ~100 Hz de control
}
