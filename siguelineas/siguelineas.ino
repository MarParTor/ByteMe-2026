// ============================================================
//  SIGUELINEAS CON AUTOCALIBRACIÓN DINÁMICA
//  6 sensores IR reflectivos (OUT0..OUT5 en A0..A5)
//  Convención del hardware:
//    1023 = negro más puro   (mucha reflexión IR absorbida)
//       0 = blanco más puro  (mucha reflexión IR reflejada)
// ============================================================

// ── PINES ────────────────────────────────────────────────────
const int sensorPins[6] = { A0, A1, A2, A3, A4, A5 };
const int NUM_SENSORS   = 6;

// ── PINES DE MOTORES ─────────────────────────────────────────
// Se asume un driver tipo L298N o similar con:
//   - PIN_ENA / PIN_ENB: PWM de velocidad (Enable A / B)
//   - PIN_IN1 / PIN_IN2: dirección motor izquierdo
//   - PIN_IN3 / PIN_IN4: dirección motor derecho
const int PIN_ENA = 5;   // PWM motor izquierdo
const int PIN_IN1 = 4;   // Motor izq. adelante
const int PIN_IN2 = 3;   // Motor izq. atrás  (PWM en este pin también)
const int PIN_ENB = 6;   // PWM motor derecho
const int PIN_IN3 = 7;   // Motor der. adelante
const int PIN_IN4 = 8;   // Motor der. atrás

// ── PARÁMETROS DE VELOCIDAD ──────────────────────────────────
const int VEL_BASE    = 160;  // 0-255: velocidad crucero
const int VEL_GIRO    = 220;  // velocidad del lado externo al girar
const int VEL_MINIMA  = 60;   // velocidad mínima del lado interno al girar

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
const int UMBRAL_NEGRO = 50;

// ============================================================
//  FUNCIONES DE MOTORES
// ============================================================

/**
 * motorIzq(vel)
 * Controla el motor izquierdo.
 * vel > 0 → adelante, vel < 0 → atrás, vel = 0 → freno.
 * |vel| se mapea al rango PWM 0-255.
 */
void motorIzq(int vel) {
  vel = constrain(vel, -255, 255);
  if (vel > 0) {
    digitalWrite(PIN_IN1, HIGH);
    digitalWrite(PIN_IN2, LOW);
    analogWrite(PIN_ENA, vel);
  } else if (vel < 0) {
    digitalWrite(PIN_IN1, LOW);
    digitalWrite(PIN_IN2, HIGH);
    analogWrite(PIN_ENA, -vel);
  } else {
    // Freno: ambas entradas HIGH cortocircuitan el motor
    digitalWrite(PIN_IN1, HIGH);
    digitalWrite(PIN_IN2, HIGH);
    analogWrite(PIN_ENA, 0);
  }
}

/**
 * motorDer(vel)
 * Igual que motorIzq pero para el motor derecho.
 */
void motorDer(int vel) {
  vel = constrain(vel, -255, 255);
  if (vel > 0) {
    digitalWrite(PIN_IN3, HIGH);
    digitalWrite(PIN_IN4, LOW);
    analogWrite(PIN_ENB, vel);
  } else if (vel < 0) {
    digitalWrite(PIN_IN3, LOW);
    digitalWrite(PIN_IN4, HIGH);
    analogWrite(PIN_ENB, -vel);
  } else {
    digitalWrite(PIN_IN3, HIGH);
    digitalWrite(PIN_IN4, HIGH);
    analogWrite(PIN_ENB, 0);
  }
}

/**
 * detener()
 * Para ambos motores inmediatamente.
 */
void detener() {
  motorIzq(0);
  motorDer(0);
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
 * De este modo, si el robot pasa por zonas más oscuras o más
 * claras que las vistas hasta ahora, los límites se amplían
 * automáticamente sin necesidad de reiniciar.
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
  // map() de Arduino hace la interpolación lineal
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
 * error < 0 → línea a la izquierda → el robot gira izquierda
 * error > 0 → línea a la derecha   → el robot gira derecha
 *
 * La corrección se suma/resta a la velocidad base de cada motor.
 * Cuanto mayor |error|, más agresivo el giro.
 */
void seguirLinea(int error) {
  // Ganancia proporcional: ajusta según la pista real
  const float Kp = 20.0;

  int correccion = (int)(Kp * error);

  int velIzq = VEL_BASE + correccion;
  int velDer = VEL_BASE - correccion;

  // Limitar al rango válido de PWM
  velIzq = constrain(velIzq, -255, 255);
  velDer = constrain(velDer, -255, 255);

  motorIzq(velIzq);
  motorDer(velDer);
}

/**
 * manejarLineaPerdida()
 * Estrategia básica cuando no se detecta línea:
 * el robot frena para no perderse más.
 * Puedes sustituir esto por una búsqueda giratoria si prefieres.
 */
void manejarLineaPerdida() {
  detener();
}

// ============================================================
//  SETUP
// ============================================================
void setup() {
  Serial.begin(9600);

  // Configurar pines de motores como salida
  pinMode(PIN_ENA, OUTPUT);
  pinMode(PIN_IN1, OUTPUT);
  pinMode(PIN_IN2, OUTPUT);
  pinMode(PIN_ENB, OUTPUT);
  pinMode(PIN_IN3, OUTPUT);
  pinMode(PIN_IN4, OUTPUT);

  detener();
  inicializarCalibracion();

  // ── FASE DE CALIBRACIÓN INICIAL ──────────────────────────
  // Antes de moverse, el robot toma LECTURAS_INIT muestras
  // estático para establecer un rango base de calibración.
  // En este punto conviene mover el robot manualmente sobre
  // blanco y negro si es posible, pero si no, con quedarse
  // quieto ya siembra algo útil.
  Serial.println("Calibrando...");
  for (int k = 0; k < LECTURAS_INIT; k++) {
    int raw[NUM_SENSORS];
    for (int i = 0; i < NUM_SENSORS; i++) {
      raw[i] = analogRead(sensorPins[i]);
    }
    actualizarCalibracion(raw);
    delay(5);
  }
  Serial.println("Calibración inicial lista. Iniciando...");
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
    seguirLinea(error);
  }

  // 6. Debug opcional por Serial (puedes comentarlo en producción)
  Serial.print("Cal[min/max]: ");
  for (int i = 0; i < NUM_SENSORS; i++) {
    Serial.print(calMin[i]); Serial.print("/");
    Serial.print(calMax[i]); Serial.print("  ");
  }
  Serial.print("| Norm: ");
  for (int i = 0; i < NUM_SENSORS; i++) {
    Serial.print(norm[i]); Serial.print(" ");
  }
  Serial.print("| Error: ");
  Serial.println(error);

  delay(10);  // ~100 Hz de control
}
