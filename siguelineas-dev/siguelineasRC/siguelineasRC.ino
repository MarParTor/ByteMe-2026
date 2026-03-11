// ============================================================
//  Robot Siguelíneas — PID + Recuperación  |  L298N  |  v3
//
//  Hardware:
//    · 6 sensores IR QTR-8RC → pines A0–A5
//    · L298N Motor izq: IN1=2, E1=3(PWM)
//    · L298N Motor der: IN2=4, E2=5(PWM)
//    · LEDON array QTR → pin 8
// ============================================================

// ---- Sensores ----
const uint8_t NUM_SENSORS = 6;
const uint8_t SENSOR_PINS[NUM_SENSORS] = {A5, A4, A3, A2, A1, A0};
const uint8_t LEDON_PIN = 8;

// ---- Motores L298N ----
const uint8_t M1_IN = 2;   // dirección izq
const uint8_t M1_E  = 3;   // PWM izq
const uint8_t M2_IN = 5;   // dirección der
const uint8_t M2_E  = 4;   // PWM der

// ---- Velocidades ----
const int BASE_SPEED = 220;

// ---- PID ----
float Kp = 4.0;
float Ki = 0.1; //0.5
float Kd = 4;
float integral  = 0.0;
float lastError = 0.0;

// ---- Sensores calibración ----
uint16_t sensorMin[NUM_SENSORS];
uint16_t sensorMax[NUM_SENSORS];
const uint16_t RC_TIMEOUT = 2500;
const uint16_t THRESHOLD  = 500;

// ---- Recuperación de línea ----
//  lastSide guarda SIEMPRE hacia qué lado se vio la línea por última vez
//  Se actualiza en CADA lectura donde se encuentra línea (sin zona muerta)
int8_t lastSide  = 1;       // 1 = derecha, -1 = izquierda
bool   lineLost  = false;

// Velocidades del arco de búsqueda
const int SEARCH_OUTER =  220;   // motor exterior: máximo
const int SEARCH_INNER = -220;   // motor interior: máximo reversa

// ============================================================
//  MOTORES
// ============================================================
void motorRight(int speed) {
  speed = constrain(speed, -255, 255);
  digitalWrite(M1_IN, speed >= 0 ? HIGH : LOW);
  analogWrite(M1_E, abs(speed));
}

void motorLeft(int speed) {
  speed = constrain(speed, -255, 255);
  digitalWrite(M2_IN, speed >= 0 ? HIGH : LOW);
  analogWrite(M2_E, abs(speed));
}

// ============================================================
//  LECTURA RAW QTR-8RC
// ============================================================
void readRawRC(uint16_t *raw) {
  for (uint8_t i = 0; i < NUM_SENSORS; i++) {
    pinMode(SENSOR_PINS[i], OUTPUT);
    digitalWrite(SENSOR_PINS[i], HIGH);
  }
  delayMicroseconds(10);
  for (uint8_t i = 0; i < NUM_SENSORS; i++) {
    pinMode(SENSOR_PINS[i], INPUT);
    raw[i] = RC_TIMEOUT;
  }

  bool     done[NUM_SENSORS] = {};
  uint8_t  remaining = NUM_SENSORS;
  uint32_t t0 = micros();

  while (remaining > 0) {
    uint32_t dt = micros() - t0;
    if (dt >= RC_TIMEOUT) break;
    for (uint8_t i = 0; i < NUM_SENSORS; i++) {
      if (!done[i] && digitalRead(SENSOR_PINS[i]) == LOW) {
        raw[i]  = (uint16_t)dt;
        done[i] = true;
        remaining--;
      }
    }
  }
}

// ============================================================
//  LECTURA NORMALIZADA  0 (blanco) … 1000 (negro)
// ============================================================
void readNorm(uint16_t *val) {
  uint16_t raw[NUM_SENSORS];
  readRawRC(raw);
  for (uint8_t i = 0; i < NUM_SENSORS; i++) {
    uint16_t r = constrain(raw[i], sensorMin[i], sensorMax[i]);
    uint32_t range = sensorMax[i] - sensorMin[i];
    val[i] = (range == 0) ? 0
           : (uint16_t)(((uint32_t)(r - sensorMin[i]) * 1000UL) / range);
  }
}

// ============================================================
//  POSICIÓN DE LÍNEA  −2500 … +2500
// ============================================================
int16_t getPosition(uint16_t *val, bool &found) {
  readNorm(val);

  uint32_t wSum = 0, sum = 0;
  found = false;

  for (uint8_t i = 0; i < NUM_SENSORS; i++) {
    if (val[i] > THRESHOLD) found = true;
    uint32_t pos = (uint32_t)i * 1000UL;
    wSum += (uint32_t)val[i] * pos;
    sum  += val[i];
  }

  if (!found) return 0;
  return (int16_t)((int32_t)(wSum / sum) - 2500L);
}

// ============================================================
//  CALIBRACIÓN  (~3 s)
// ============================================================
void calibrate() {
  for (uint8_t i = 0; i < NUM_SENSORS; i++) {
    sensorMin[i] = RC_TIMEOUT;
    sensorMax[i] = 0;
  }

  uint32_t t0 = millis();
  while (millis() - t0 < 3000) {
    uint32_t t = millis() - t0;
    if      (t < 750)  { motorLeft( 80); motorRight(-80); }
    else if (t < 2250) { motorLeft(-80); motorRight( 80); }
    else               { motorLeft( 80); motorRight(-80); }

    uint16_t raw[NUM_SENSORS];
    readRawRC(raw);
    for (uint8_t i = 0; i < NUM_SENSORS; i++) {
      if (raw[i] < sensorMin[i]) sensorMin[i] = raw[i];
      if (raw[i] > sensorMax[i]) sensorMax[i] = raw[i];
    }
  }
  motorLeft(0);
  motorRight(0);
  delay(500);
}

// ============================================================
//  SETUP
// ============================================================
void setup() {
  delay(4000);

  pinMode(LEDON_PIN, OUTPUT);
  digitalWrite(LEDON_PIN, HIGH);

  pinMode(M1_IN, OUTPUT); pinMode(M1_E, OUTPUT);
  pinMode(M2_IN, OUTPUT); pinMode(M2_E, OUTPUT);

  motorLeft(0);
  motorRight(0);
  delay(500);

  calibrate();
}

// ============================================================
//  LOOP
// ============================================================
void loop() {
  uint16_t sens[NUM_SENSORS];
  bool found;

  int16_t pos = getPosition(sens, found);

  // ==========================================================
  //  LÍNEA ENCONTRADA → PID
  // ==========================================================
  if (found) {

    // Resetear PID si venimos de búsqueda
    if (lineLost) {
      integral  = 0.0;
      lastError = 0.0;
      lineLost  = false;
    }

    // Actualizar lado SIEMPRE que hay línea (sin zona muerta)
    if (pos < 0) lastSide = -1;
    if (pos > 0) lastSide =  1;
    // Si pos == 0 exacto, mantiene el último lado

    // PID
    float error = (float)pos;
    integral   += error;
    integral    = constrain(integral, -5000.0f, 5000.0f);
    float deriv = error - lastError;
    lastError   = error;

    float correction = Kp * error + Ki * integral + Kd * deriv;


    // Motores: 0..255 (sin reversa en seguimiento normal)
    int dynamicSpeed = BASE_SPEED - abs(error) * 0.04;
    dynamicSpeed = constrain(dynamicSpeed, 150, BASE_SPEED);

    int sL = constrain((int)(dynamicSpeed + correction), -80, 255);
    int sR = constrain((int)(dynamicSpeed - correction), -80, 255);
    //int sL = constrain((int)(dynamicSpeed + correction), 0, 255);
    //int sR = constrain((int)(dynamicSpeed - correction), 0, 255);

    motorLeft(sL);
    motorRight(sR);

  // ==========================================================
  //  LÍNEA PERDIDA → GIRAR EN ARCO HASTA ENCONTRARLA
  //  Sin timeout. Sin cambio de dirección. Gira para siempre.
  // ==========================================================
  } else {

    lineLost = true;

    // lastSide indica dónde se fue la línea → girar hacia ahí
    if (lastSide > 0) {
      // Línea a la derecha → motor izq avanza, motor der reversa
      motorLeft(SEARCH_OUTER);
      motorRight(SEARCH_INNER);
    } else {
      // Línea a la izquierda → motor der avanza, motor izq reversa
      motorLeft(SEARCH_INNER);
      motorRight(SEARCH_OUTER);
    }
  }
}
