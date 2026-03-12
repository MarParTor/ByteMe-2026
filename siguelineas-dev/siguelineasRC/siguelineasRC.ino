//  Robot Siguelíneas — PID + Recuperación  |  L298N  |  v3
//  Ahora espera a recibir "e" antes de empezar

#include <SoftwareSerial.h>
SoftwareSerial BT(10, 11);
#define BT_BAUD 9600

// --- Sensores ---
const uint8_t NUM_SENSORS = 6;
const uint8_t SENSOR_PINS[NUM_SENSORS] = {A5, A4, A3, A2, A1, A0};
const uint8_t LEDON_PIN = 8;

// --- Motores ---
const uint8_t M1_IN = 2;   // dirección izq
const uint8_t M1_E  = 3;   // PWM izq
const uint8_t M2_IN = 5;   // dirección der
const uint8_t M2_E  = 4;   // PWM der

// --- Velocidades ---
const int BASE_SPEED = 220;

// --- PID ---
float Kp = 4.0;
float Ki = 0.1;
float Kd = 4;
float integral  = 0.0;
float lastError = 0.0;

// --- Sensores calibración ---
uint16_t sensorMin[NUM_SENSORS];
uint16_t sensorMax[NUM_SENSORS];
const uint16_t RC_TIMEOUT = 2500;
const uint16_t THRESHOLD  = 500;

// --- Recuperación de línea ---
int8_t lastSide  = 1;       // 1 = derecha, -1 = izquierda
bool   lineLost  = false;

// Velocidades del arco de búsqueda
const int SEARCH_OUTER =  220;   
const int SEARCH_INNER = -220;   

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
//  SENSORES
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

  bool done[NUM_SENSORS] = {};
  uint8_t remaining = NUM_SENSORS;
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

void readNorm(uint16_t *val) {
  uint16_t raw[NUM_SENSORS];
  readRawRC(raw);
  for (uint8_t i = 0; i < NUM_SENSORS; i++) {
    uint16_t r = constrain(raw[i], sensorMin[i], sensorMax[i]);
    uint32_t range = sensorMax[i] - sensorMin[i];
    val[i] = (range == 0) ? 0 : (uint16_t)(((uint32_t)(r - sensorMin[i]) * 1000UL) / range);
  }
}

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

// Calibración
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
  motorLeft(0); motorRight(0);
  delay(500);
}

// ============================================================
//  ESPERAR SEÑAL POR BLUETOOTH
// ============================================================
void waitForStart() {
  Serial.println("Esperando 'e' para empezar...");
  BT.println("Esperando 'e' para empezar...");
  while (true) {
    if (BT.available()) {
      char c = BT.read();
      if (c == 'e') {
        Serial.println("¡Comenzando!");
        BT.println("¡Comenzando!");
        return;
      }
    }
  }
}

// ============================================================
//  SETUP
// ============================================================
void setup() {
  delay(1000);
  pinMode(LEDON_PIN, OUTPUT);
  digitalWrite(LEDON_PIN, HIGH);

  pinMode(M1_IN, OUTPUT); pinMode(M1_E, OUTPUT);
  pinMode(M2_IN, OUTPUT); pinMode(M2_E, OUTPUT);

  motorLeft(0); motorRight(0);

  Serial.begin(9600);
  BT.begin(BT_BAUD);

  calibrate();    // calibración de sensores
  waitForStart(); // espera la señal 'e'
}

// ============================================================
//  LOOP PRINCIPAL
// ============================================================
void loop() {
  uint16_t sens[NUM_SENSORS];
  bool found;
  int16_t pos = getPosition(sens, found);

  if (found) {
    if (lineLost) { integral = 0.0; lastError = 0.0; lineLost = false; }
    if (pos < 0) lastSide = -1;
    if (pos > 0) lastSide = 1;

    float error = (float)pos;
    integral += error;
    integral = constrain(integral, -5000.0f, 5000.0f);
    float deriv = error - lastError;
    lastError = error;

    float correction = Kp * error + Ki * integral + Kd * deriv;
    int curveBoost = 0;
    if (sens[0] > 800 && sens[5] < 400) curveBoost = -120;
    else if (sens[5] > 800 && sens[0] < 400) curveBoost = 120;
    correction += curveBoost;

    int dynamicSpeed = BASE_SPEED - abs(error) * 0.04;
    dynamicSpeed = constrain(dynamicSpeed, 150, BASE_SPEED);

    int sL = constrain((int)(dynamicSpeed + correction), -80, 255);
    int sR = constrain((int)(dynamicSpeed - correction), -80, 255);

    motorLeft(sL); motorRight(sR);

  } else {
    lineLost = true;
    if (lastSide > 0) { motorLeft(SEARCH_OUTER); motorRight(SEARCH_INNER); }
    else               { motorLeft(SEARCH_INNER); motorRight(SEARCH_OUTER); }
  }
}