const uint8_t TRIG_PIN = 6;
const uint8_t ECHO_PIN = 7;

const uint8_t M1_IN = 2;
const uint8_t M1_E  = 3;
const uint8_t M2_IN = 5;
const uint8_t M2_E  = 4;

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
void atacar(){
  motorLeft(255);
  motorRight(255);
}
void buscar(){
  static uint32_t lastTurn = 0;
  static int8_t dir = 1;
  const uint32_t ZIGZAG_INTERVAL = 1000; // ms entre giros, ajusta a tu gusto

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

void setup() {
  Serial.begin(9600);
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  pinMode(M1_IN, OUTPUT); pinMode(M1_E, OUTPUT);
  pinMode(M2_IN, OUTPUT); pinMode(M2_E, OUTPUT);
  motorLeft(0);
  motorRight(0);
}

void loop() {
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
    Serial.println(">> ENEMIGO DETECTADO — atacando");
    atacar();
  } else {
    buscar();
  }

  delay(200);
}