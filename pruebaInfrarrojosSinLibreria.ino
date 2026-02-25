// Pines analógicos donde conectas OUT0...OUT7
const int sensorPins[8] = {
  A0, A1, A2, A3, A4, A5
};

void setup() {
  Serial.begin(9600);
}

void loop() {
  for (int i = 0; i < 8; i++) {
    int value = analogRead(sensorPins[i]); // Lee el ADC
    Serial.print("Sensor ");
    Serial.print(i);
    Serial.print(": ");
    Serial.println(value);
  }
  Serial.println();
  delay(100); // Ajusta al ritmo que necesites
}