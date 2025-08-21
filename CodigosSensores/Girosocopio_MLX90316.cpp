/*

Ligações SPI com ESP32

| MLX90316 (SPI)   | ESP32   |
| ---------------- | ------- |
| VDD              | 3.3V    |
| GND              | GND     |
| MISO             | GPIO 19 |
| SCLK             | GPIO 18 |
| CS (chip select) | GPIO 5  |


*/



#include <SPI.h>

const int CS_PIN = 5; // Chip Select do MLX90316

void setup() {
  Serial.begin(115200);
  SPI.begin(18, 19, 23, CS_PIN); // SCK, MISO, MOSI, SS
  pinMode(CS_PIN, OUTPUT);
  digitalWrite(CS_PIN, HIGH); // Desativa o sensor inicialmente
}

uint16_t readAngleRaw() {
  digitalWrite(CS_PIN, LOW);
  delayMicroseconds(1); // Pequeno delay para garantir estabilidade
  
  uint16_t result = SPI.transfer16(0x0000); // Envia comando nulo e lê resposta

  digitalWrite(CS_PIN, HIGH);
  return result & 0x3FFF; // Os 14 bits menos significativos contêm o ângulo
}

void loop() {
  uint16_t rawAngle = readAngleRaw();
  float angleDeg = (rawAngle / 16383.0) * 360.0; // Converte para graus (0 a 360)

  Serial.print("Ângulo (raw): ");
  Serial.print(rawAngle);
  Serial.print(" | Ângulo (°): ");
  Serial.println(angleDeg, 2);

  delay(200);
}