// ============================================================
//  SIGUELINEAS - MODO DEBUG
//  Solo lectura y autocalibración, sin control de motores.
//  Imprime por Serial los valores normalizados (0=blanco, 100=negro)
// ============================================================

const int sensorPins[6] = { A0, A1, A2, A3, A4, A5 };
const int NUM_SENSORS   = 6;

int calMin[NUM_SENSORS];
int calMax[NUM_SENSORS];

const int LECTURAS_INIT = 400;

void inicializarCalibracion() {
  for (int i = 0; i < NUM_SENSORS; i++) {
    calMin[i] = 1023;
    calMax[i] = 0;
  }
}

void actualizarCalibracion(int valores[]) {
  for (int i = 0; i < NUM_SENSORS; i++) {
    if (valores[i] < calMin[i]) calMin[i] = valores[i];
    if (valores[i] > calMax[i]) calMax[i] = valores[i];
  }
}

int normalizar(int valor, int sensor) {
  if (calMax[sensor] == calMin[sensor]) return 50;
  return map(valor, calMin[sensor], calMax[sensor], 0, 100);
}

void setup() {
  Serial.begin(9600);
  inicializarCalibracion();

  Serial.println("Calibrando...");
  for (int k = 0; k < LECTURAS_INIT; k++) {
    int raw[NUM_SENSORS];
    for (int i = 0; i < NUM_SENSORS; i++) raw[i] = analogRead(sensorPins[i]);
    actualizarCalibracion(raw);
    delay(5);
  }
  Serial.println("Listo.\n");
}

void loop() {
  int raw[NUM_SENSORS];
  for (int i = 0; i < NUM_SENSORS; i++) raw[i] = analogRead(sensorPins[i]);

  actualizarCalibracion(raw);

  // Imprimir array de valores normalizados
  Serial.print("[ ");
  for (int i = 0; i < NUM_SENSORS; i++) {
    int norm = normalizar(raw[i], i);
    Serial.print(norm);
    if (i < NUM_SENSORS - 1) Serial.print(", ");
  }
  Serial.println(" ]");

  delay(100);
}
