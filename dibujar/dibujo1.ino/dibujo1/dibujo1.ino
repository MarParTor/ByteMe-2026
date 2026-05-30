// ============================================================
//  PINES (igual que tu código funcional)
// ============================================================

#define PIN_IN1 2
#define PIN_E1  3
#define PIN_IN2 5
#define PIN_E2  4

// Velocidad
int velocidad = 240;

// TIEMPOS
int tiempo_10cm = 500;
int tiempo_giro_90 = 400;
int tiempo_giro_120 = 550;

// ============================================================
//  MOTORES (igual que código que funciona)
// ============================================================

void motorIzq(int vel) {
  vel = constrain(vel, -255, 255);
  if (vel > 0) {
    digitalWrite(PIN_IN1, HIGH);
    analogWrite(PIN_E1, vel);
  } else if (vel < 0) {
    digitalWrite(PIN_IN1, LOW);
    analogWrite(PIN_E1, -vel);
  } else {
    analogWrite(PIN_E1, 0);
  }
}

void motorDer(int vel) {
  vel = constrain(vel, -255, 255);
  if (vel > 0) {
    digitalWrite(PIN_IN2, HIGH);
    analogWrite(PIN_E2, vel);
  } else if (vel < 0) {
    digitalWrite(PIN_IN2, LOW);
    analogWrite(PIN_E2, -vel);
  } else {
    analogWrite(PIN_E2, 0);
  }
}

void detener() {
  analogWrite(PIN_E1, 0);
  analogWrite(PIN_E2, 0);
}

// ============================================================
//  MOVIMIENTOS BÁSICOS
// ============================================================

void avanzar(int tiempo) {
  motorIzq(velocidad);
  motorDer(velocidad);
  delay(tiempo);
  detener();
}

void girarDerecha(int tiempo) {
  motorIzq(velocidad);
  motorDer(-velocidad);
  delay(tiempo);
  detener();
}

void girarIzquierda(int tiempo) {
  motorIzq(-velocidad);
  motorDer(velocidad);
  delay(tiempo);
  detener();
}

// ============================================================
//  FORMAS
// ============================================================

void dibujarCuadrado() {
  for (int i = 0; i < 4; i++) {
    avanzar(tiempo_10cm * 2);
    girarDerecha(tiempo_giro_90);
  }
}

void dibujarTriangulo() {
  for (int i = 0; i < 3; i++) {
    avanzar(tiempo_10cm * 2.5);
    girarDerecha(tiempo_giro_120);
  }
}

void dibujarRectangulo() {
  for (int i = 0; i < 2; i++) {
    avanzar(tiempo_10cm * 3);
    girarDerecha(tiempo_giro_90);
    avanzar(tiempo_10cm);
    girarDerecha(tiempo_giro_90);
  }
}

// ============================================================
//  SETUP / LOOP
// ============================================================

void setup() {
  pinMode(PIN_IN1, OUTPUT);
  pinMode(PIN_E1, OUTPUT);
  pinMode(PIN_IN2, OUTPUT);
  pinMode(PIN_E2, OUTPUT);
}

void loop() {
  avanzar(tiempo_10cm);
  delay(2000);

  // dibujarCuadrado();
  // delay(2000);

  // dibujarTriangulo();
  // delay(2000);

  // dibujarRectangulo();
  // delay(5000);
}