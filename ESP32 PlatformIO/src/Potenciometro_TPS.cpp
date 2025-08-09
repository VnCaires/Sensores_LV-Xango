/*

VCC (TPS) → 3.3V do ESP32

GND (TPS) → GND do ESP32

Sinal (TPS) → GPIO 34 (ou outro ADC)


*/

// Sensor TPS (Potenciômetro) - Leitura do pedal
const int tpsPin = 34; // Pino ADC do ESP32 conectado ao TPS
int tpsValue = 0;

void setup() {
  Serial.begin(115200);
  analogReadResolution(12); // Resolução de 12 bits (0-4095) padrão no ESP32
}

void loop() {
  tpsValue = analogRead(tpsPin); // Lê o valor do potenciômetro
  float voltage = (tpsValue / 4095.0) * 3.3; // Converte para tensão

  // Mostra valor bruto e a tensão correspondente
  Serial.print("TPS (raw): ");
  Serial.print(tpsValue);
  Serial.print(" | Tensão: ");
  Serial.print(voltage, 2);
  Serial.println(" V");

  delay(200); // Aguarda 200ms
}
