/*

O INA226 (Texas Instruments) é um conversor analógico-digital de 16 bits dedicado a medir:

    Grandeza               |	      Faixa típica (com VBUS=36 V)
--------------------------------------------------------------------------
Tensão no barramento VBUS	|            0 – 36 V
Queda no shunt VSHUNT	    |           ±81,92 mV
Corrente I	                |        Definida pelo valor do resistor shunt e calibração
Potência P	                |        Calculada internamente (I × VBUS)

Ele amostra duas tensões (barramento e shunt), converte em corrente e potência e dispõe de alarme programável para sobrecorrente, sobretensão ou sobrepotência. A interface é I²C até 400 kHz.


##########################################################################################

Ligação típica:

INA226	        Conexão	                                Observação
IN+	       Lado “barramento” do shunt	            Próximo ao polo negativo do pack
IN-        Lado “carga” do shunt	                -
SCL/SDA	   GPIO 22 / GPIO 21 (ESP32 padrão I²C)	    Pull-ups 4 k7 a 3 V3
A0/A1	   Endereço I²C (GND/3V3)	                Permite até 4 dispositivos
VCC	       3,0 – 3,6 V	                            Mesmo 3 V3 do ESP32
GND	       GND comum LV



Shunt      IN+ ───\/\/─── IN-  (INA226)
Pack(-) ──┘                └─> para inversor / BMS



Calibração e equações
Para que o INA226 traduza VSHUNT em corrente, você grava o registrador CALIBRATION com um valor calculado:

ini
Copiar
Editar
CAL = 0.00512 / (Current_LSB × R_SHUNT)
R_SHUNT — resistência do shunt (Ω)

Current_LSB — passo desejado de corrente (A/bit).
Convém escolher um Current_LSB que produza um número inteiro (CAL ≤ 32 767).

Exemplo (pack HV de 400 A pico, shunt 0,5 mΩ):

Faixa de corrente desejada ≈ ±400 A → escolha Current_LSB = 0,01 A

CAL = 0,00512 / (0,01 × 0,0005) = 1024


*/


/*
 * INA226 + ESP32: medição de corrente, tensão e potência
 * Canais I2C padrão: SDA=GPIO21, SCL=GPIO22 (400 kHz)
 */
#include <Arduino.h>
#include <Wire.h>

// ---------- Configurações do sistema ----------
const uint8_t INA226_ADDR = 0x40;      // A1=A0=GND
const float   R_SHUNT     = 0.0005;    // 0,5 mΩ
const float   CURRENT_LSB = 0.01;      // 10 mA/bit
const uint16_t CAL_VALUE  = 1024;      // calculado

// Registros
#define INA_REG_CONFIG      0x00
#define INA_REG_SHUNT       0x01
#define INA_REG_BUS         0x02
#define INA_REG_POWER       0x03
#define INA_REG_CURRENT     0x04
#define INA_REG_CALIB       0x05

void writeRegister16(uint8_t reg, uint16_t value) {
  Wire.beginTransmission(INA226_ADDR);
  Wire.write(reg);
  Wire.write(value >> 8);
  Wire.write(value & 0xFF);
  Wire.endTransmission();
}

uint16_t readRegister16(uint8_t reg) {
  Wire.beginTransmission(INA226_ADDR);
  Wire.write(reg);
  Wire.endTransmission(false);       // restart
  Wire.requestFrom(INA226_ADDR, (uint8_t)2);
  uint16_t value = ((uint16_t)Wire.read() << 8) | Wire.read();
  return value;
}

void setup() {
  Serial.begin(115200);
  Wire.begin(21, 22, 400000);        // SDA, SCL, 400 kHz

  // 1. Configuração: média 16, conversão 1,1 ms, modo contínuo (shunt+bus)
  uint16_t cfg = 0b001 << 9   |      // AVG = 16
                 0b100 << 6   |      // VBUSCT = 1.1 ms
                 0b100 << 3   |      // VSHCT  = 1.1 ms
                 0b111;              // MODE = Shunt+Bus, contínuo
  writeRegister16(INA_REG_CONFIG, cfg);

  // 2. Calibração
  writeRegister16(INA_REG_CALIB, CAL_VALUE);

  Serial.println("INA226 inicializado.");
}

void loop() {
  // 3. Leitura dos registros
  int16_t rawShunt   = (int16_t)readRegister16(INA_REG_SHUNT);
  uint16_t rawBus    = readRegister16(INA_REG_BUS);
  int16_t rawCurrent = (int16_t)readRegister16(INA_REG_CURRENT);
  uint16_t rawPower  = readRegister16(INA_REG_POWER);

  // 4. Conversões
  float vShunt = rawShunt * 2.5e-6;           // 2,5 µV/LSB
  float vBus   = rawBus  * 1.25e-3;           // 1,25 mV/LSB
  float current = rawCurrent * CURRENT_LSB;   // A
  float power   = rawPower * 25 * CURRENT_LSB;// W

  // 5. Saída
  Serial.printf("Bus: %.3f V  Shunt: %.3f mV  I: %.2f A  P: %.1f W\n",
                vBus, vShunt * 1e3, current, power);

  delay(100);  // 10 Hz
}