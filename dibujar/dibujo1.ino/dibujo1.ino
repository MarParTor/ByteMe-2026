// Pines motores
int IN1 = 8;
int IN2 = 9;
int IN3 = 10;
int IN4 = 11;

// Velocidad (0–255 si usas PWM, aquí simple ON/OFF)
int velocidad = 200;

// TIEMPOS (ajustar según tu robot)
int tiempo_10cm = 500;     // tiempo para avanzar 10 cm
int tiempo_giro_90 = 400;  // tiempo para girar 90°
int tiempo_giro_120 = 550; // tiempo para girar 120°

// ================= FUNCIONES BÁSICAS =================

void setup() {
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
}

void loop() {

  dibujarCuadrado();
  delay(2000);

  dibujarTriangulo();
  delay(2000);

  dibujarRectangulo();
  delay(5000);

}

// Avanzar
void avanzar(int tiempo) {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  delay(tiempo);
  parar();
}

// Girar derecha
void girarDerecha(int tiempo) {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  delay(tiempo);
  parar();
}

// Parar
void parar() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  delay(500);
}

// ================= FIGURAS =================

// Cuadrado 20x20 cm
void dibujarCuadrado() {
  for (int i = 0; i < 4; i++) {
    avanzar(tiempo_10cm * 2); // 20 cm
    girarDerecha(tiempo_giro_90);
  }
}

// Triángulo equilátero (25 cm lado)
void dibujarTriangulo() {
  for (int i = 0; i < 3; i++) {
    avanzar(tiempo_10cm * 2.5); // 25 cm
    girarDerecha(tiempo_giro_120);
  }
}

// Rectángulo 30x10 cm
void dibujarRectangulo() {
  for (int i = 0; i < 2; i++) {
    avanzar(tiempo_10cm * 3);   // 30 cm
    girarDerecha(tiempo_giro_90);
    avanzar(tiempo_10cm * 1);   // 10 cm
    girarDerecha(tiempo_giro_90);
  }
}
