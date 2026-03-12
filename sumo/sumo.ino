// bluetooth
#include <SoftwareSerial.h>
SoftwareSerial BT(10, 11);
#define BT_BAUD 9600

// ---- Ultrasonido ----
const uint8_t TRIG_PIN = 6;
const uint8_t ECHO_PIN = 7;

// ---- Motores ----
const uint8_t M1_IN = 2;
const uint8_t M1_E  = 3;
const uint8_t M2_IN = 5;
const uint8_t M2_E  = 4;

// ---- Sensores QTR ----
const uint8_t NUM_SENSORS = 6;
const uint8_t SENSOR_PINS[NUM_SENSORS] = {A5, A4, A3, A2, A1, A0};
const uint8_t LEDON_PIN = 8;

const uint16_t RC_TIMEOUT = 2500;
const uint16_t BLACK_THRESHOLD = 1600;

// ============================================================
// MOTORES
// ============================================================

void motorRight(int speed) {
  speed = constrain(speed, 255, -255);
  if (speed == 0) {
    analogWrite(M1_E, 0);
    return;
  }
  digitalWrite(M1_IN, speed >= 0 ? HIGH : LOW);
  analogWrite(M1_E, abs(speed));
}

void motorLeft(int speed) {
  speed = constrain(speed, 255, -255);
  if (speed == 0) {
    analogWrite(M2_E, 0);
    return;
  }
  digitalWrite(M2_IN, speed >= 0 ? HIGH : LOW);
  analogWrite(M2_E, abs(speed));
}

// ============================================================
// LECTURA RAW QTR
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

        raw[i] = dt;
        done[i] = true;
        remaining--;
      }
    }
  }
}

// ============================================================
// DETECCION LINEA NEGRA
// ============================================================

bool lineaNegra(uint16_t *rawValues) {

  // rawValues debe tener espacio para NUM_SENSORS
  readRawRC(rawValues);

  for (uint8_t i = 0; i < NUM_SENSORS; i++) {

    if (rawValues[i] > BLACK_THRESHOLD) {
      return true;
    }
  }

  return false;
}

// COMPORTAMIENTOS

void atacar(){
  motorLeft(255);
  motorRight(255);
}

void buscar(){

  static uint32_t lastTurn = 0;
  static int8_t dir = 1;
  const uint32_t ZIGZAG_INTERVAL = 1000;

  uint32_t now = millis();

  if (now - lastTurn >= ZIGZAG_INTERVAL) {
    dir = -dir;
    lastTurn = now;
  }

  motorLeft(255);
  motorRight(255);

  delay(80);

  if (dir > 0) {
    motorLeft(255);
    motorRight(0);
  } else {
    motorLeft(0);
    motorRight(255);
  }

  delay(100);
}

void retroceder(){

  motorLeft(-255);
  motorRight(-255);

  delay(1100);
}
//  ESPERAR SEÑAL POR BLUETOOTH
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

void setup() {

  Serial.begin(9600);
  BT.begin(BT_BAUD);

  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  pinMode(M1_IN, OUTPUT);
  pinMode(M1_E, OUTPUT);
  pinMode(M2_IN, OUTPUT);
  pinMode(M2_E, OUTPUT);

  pinMode(LEDON_PIN, OUTPUT);
  digitalWrite(LEDON_PIN, HIGH);


  waitForStart(); // espera la señal 'e'
}

// LOOP
void loop() {

  uint16_t raw[NUM_SENSORS];

  readRawRC(raw);
  Serial.print("IR raw: ");
  for (uint8_t i = 0; i < NUM_SENSORS; i++) {
    Serial.print(raw[i]);
    Serial.print(i < NUM_SENSORS-1 ? " | " : "");
  }
  Serial.println();

  // LINEA NEGRA
  if (lineaNegra(raw)) {
    Serial.println("LINEA NEGRA DETECTADA");
    retroceder();
    return;
  }

  // -------- ULTRASONIDO --------

  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);

  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);

  digitalWrite(TRIG_PIN, LOW);

  long duration = pulseIn(ECHO_PIN, HIGH, 25000);
  float dist = duration * 0.01716;

  Serial.print("Distancia: ");
  Serial.print(dist);
  Serial.println(" cm");

  if (dist > 0 && dist < 40.0) {
    atacar();
  } else {
    buscar();
  }

  delay(200);
}