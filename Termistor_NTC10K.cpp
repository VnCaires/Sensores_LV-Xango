/*

Modelagem matemática
A forma simples usa a equação Beta (válida em intervalos moderados, 0 – 80 °C):

    1/T = 1/T0 + (1/β) · ln(R/R0)

T — temperatura absoluta (K)

T0 — temperatura de referência (298 K = 25 °C)

R0 — resistência nominal a 25 °C (10 kΩ)

β — constante Beta do termistor (– consulte a folha de dados; 3950 K é comum)


 Leitura de termistor NTC 10k em divisor de tensão
 GPIO 34 como entrada analógica

 */

#include <Arduino.h>

//------------- Configurações -------------
const int  PIN_NTC          = 34;      // ADC1_CH6
const int  ADC_RESOLUTION   = 4095;    // 12 bits
const float VREF            = 3.3;     // tensão da placa ESP32
const float R_FIXED         = 10000.0; // 10 kΩ de precisão 1%

// Coeficientes do termistor
const float T0              = 298.15;  // 25 °C em Kelvin
const float BETA            = 3950.0;  // Beta típico
const float R0              = 10000.0; // 10 kΩ

// Filtro simples (média móvel)
const uint8_t SAMPLES       = 10;
float buffer[SAMPLES];
uint8_t idx = 0;

void setup() {
  Serial.begin(115200);
  analogReadResolution(12);            // 12 bits
  analogSetAttenuation(ADC_11db);      // full-scale ≈3.3 V
}

void loop() {
  // 1. Lê ADC e converte para tensão
  uint32_t raw = analogRead(PIN_NTC);
  float v_ntc  = (raw * VREF) / ADC_RESOLUTION;

  // 2. Calcula resistência do termistor
  float r_ntc = (R_FIXED * v_ntc) / (VREF - v_ntc);

  // 3. Calcula temperatura usando equação Beta
  float inv_T = (1.0 / T0) + (1.0 / BETA) * log(r_ntc / R0);
  float tempK = 1.0 / inv_T;
  float tempC = tempK - 273.15;

  // 4. Atualiza filtro
  buffer[idx] = tempC;
  idx = (idx + 1) % SAMPLES;

  float avg = 0;
  for (uint8_t i = 0; i < SAMPLES; ++i) avg += buffer[i];
  avg /= SAMPLES;

  // 5. Saída
  Serial.printf("ADC:%4u  V:%.3f V  R:%.0f Ω  T:%.2f °C  T(filtro):%.2f °C\n",
                raw, v_ntc, r_ntc, tempC, avg);

  delay(200);  // 5 Hz
}
