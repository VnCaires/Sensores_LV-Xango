/*

  AS5600 (encoder magnético absoluto) + ESP32 via I2C
  - Lê ângulo absoluto (12 bits => 4096 passos/volta)
  - Converte para graus e radianos
  - Checa status do ímã (MD/ML/MH), lê AGC e magnitude
  - Calcula velocidade (deg/s e RPM) com tratamento de wrap-around
  - Zero de software (pressione uma condição para chamar setZeroToCurrent())


  Pinos do AS5600 (podem variar conforme o breakout):
    - VCC  -> 3V3 do ESP32
            (muitos módulos aceitam 5 V, mas como o ESP32 é 3,3 V, prefira alimentá-lo em 3V3)
    - GND  -> GND do ESP32
    - SDA  -> GPIO 21 do ESP32 (SDA padrão)
    - SCL  -> GPIO 22 do ESP32 (SCL padrão)

*/



#include <Wire.h>

// =================== Pinos/Endereço I2C ===================
static const int I2C_SDA = 21;        // altere se necessário
static const int I2C_SCL = 22;        // altere se necessário
static const uint32_t I2C_FREQ = 400000; // 400 kHz (rápido e estável)
static const uint8_t AS5600_ADDR = 0x36; // 7-bit address do AS5600

// =================== Registradores úteis ==================
// (Convenção: MSB primeiro no AS5600)
static const uint8_t REG_ZMCO      = 0x00; // contador de burns (info)
static const uint8_t REG_ZPOS_H    = 0x01; // zero position (opcional, via burn)
static const uint8_t REG_ZPOS_L    = 0x02;
static const uint8_t REG_MPOS_H    = 0x03; // pos. mínima (opcional)
static const uint8_t REG_MPOS_L    = 0x04;
static const uint8_t REG_MANG_H    = 0x05; // ângulo máx (opcional)
static const uint8_t REG_MANG_L    = 0x06;
static const uint8_t REG_CONF_H    = 0x07; // configuração (filtros, pwm, etc.)
static const uint8_t REG_CONF_L    = 0x08;

static const uint8_t REG_STATUS    = 0x0B; // bits: MD, ML, MH (detecção do imã)
static const uint8_t REG_RAW_ANG_H = 0x0C; // 12 bits não filtrado
static const uint8_t REG_RAW_ANG_L = 0x0D;
static const uint8_t REG_ANG_H     = 0x0E; // 12 bits filtrado
static const uint8_t REG_ANG_L     = 0x0F;

static const uint8_t REG_AGC       = 0x1A; // Auto Gain Control
static const uint8_t REG_MAG_H     = 0x1B; // magnitude do campo
static const uint8_t REG_MAG_L     = 0x1C;

// =================== Constantes/Utilidades =================
static const uint16_t CPR = 4096;       // Counts Per Revolution (12 bits)
static const float    DEG_PER_CNT = 360.0f / CPR;
static const float    RAD_PER_CNT = 6.283185307179586f / CPR;

struct MagnetStatus {
  bool magnetDetected; // MD: imã presente
  bool magnetTooWeak;  // ML: fraco
  bool magnetTooStrong;// MH: forte
};

struct AS5600Data {
  uint16_t angleCnt;   // 0..4095 (filtrado)
  uint16_t rawCnt;     // 0..4095 (não filtrado)
  float    angleDeg;   // 0..360
  float    angleRad;   // 0..2π
  MagnetStatus status; // estado do imã
  uint8_t  agc;        // 0..255 (ganho automático)
  uint16_t magnitude;  // unidade do sensor (relativa)
  // Dinâmica:
  float    velDeg_s;   // velocidade em graus/seg
  float    rpm;        // rotações por minuto
};

// Zero de software (offset em contagens)
int32_t zeroOffsetCnt = 0;

// Para velocidade:
uint16_t prevCnt = 0;
unsigned long prevMs = 0;

unsigned long lastPrintMs = 0;
const unsigned long PRINT_INTERVAL_MS = 200; // ajuste conforme sua taxa de telemetria

// ==================== Funções I2C base ====================
bool i2cReadBytes(uint8_t dev, uint8_t reg, uint8_t *buf, uint8_t len) {
  Wire.beginTransmission(dev);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) { // false => manter conexão p/ read
    return false;
  }
  uint8_t read = Wire.requestFrom((int)dev, (int)len);
  if (read != len) return false;
  for (uint8_t i = 0; i < len; i++) buf[i] = Wire.read();
  return true;
}

bool i2cRead8(uint8_t dev, uint8_t reg, uint8_t &val) {
  return i2cReadBytes(dev, reg, &val, 1);
}

bool i2cRead16(uint8_t dev, uint8_t regMSB, uint16_t &val) {
  uint8_t b[2];
  if (!i2cReadBytes(dev, regMSB, b, 2)) return false;
  val = ((uint16_t)b[0] << 8) | b[1]; // MSB depois LSB
  return true;
}

// ==================== Funções AS5600 ======================
bool readAngleCounts(uint16_t &cnt) {
  // ANGLE é filtrado; RAW_ANGLE é amostra "crua".
  uint16_t v;
  if (!i2cRead16(AS5600_ADDR, REG_ANG_H, v)) return false;
  cnt = v & 0x0FFF; // 12 bits
  return true;
}

bool readRawCounts(uint16_t &cnt) {
  uint16_t v;
  if (!i2cRead16(AS5600_ADDR, REG_RAW_ANG_H, v)) return false;
  cnt = v & 0x0FFF;
  return true;
}

bool readStatus(MagnetStatus &st) {
  uint8_t s;
  if (!i2cRead8(AS5600_ADDR, REG_STATUS, s)) return false;
  // Bits típicos: MD(bit5), ML(bit4), MH(bit3)
  st.magnetDetected = (s & (1 << 5)) != 0;
  st.magnetTooWeak  = (s & (1 << 4)) != 0;
  st.magnetTooStrong= (s & (1 << 3)) != 0;
  return true;
}

bool readAGC(uint8_t &agc) {
  return i2cRead8(AS5600_ADDR, REG_AGC, agc);
}

bool readMagnitude(uint16_t &mag) {
  return i2cRead16(AS5600_ADDR, REG_MAG_H, mag);
}

// Converte contagens (0..4095) aplicando zero de software
uint16_t applyZero(uint16_t cnt) {
  int32_t adj = (int32_t)cnt - zeroOffsetCnt;
  // wrap para 0..4095
  adj %= (int32_t)CPR;
  if (adj < 0) adj += CPR;
  return (uint16_t)adj;
}

// Calcula delta entre duas leituras em [-2048..+2047] (menor caminho)
int16_t diffWrapped(uint16_t now, uint16_t prev) {
  int32_t d = (int32_t)now - (int32_t)prev;
  if (d >  (int32_t)CPR/2) d -= CPR;
  if (d < -(int32_t)CPR/2) d += CPR;
  return (int16_t)d;
}

void setZeroToCurrent() {
  uint16_t cnt;
  if (readAngleCounts(cnt)) {
    zeroOffsetCnt = cnt; // faz o ângulo atual virar 0°
    Serial.println(F("[AS5600] Zero de software aplicado com sucesso."));
  } else {
    Serial.println(F("[AS5600] Falha ao ler para zerar."));
  }
}

// Coleta tudo em uma struct (inclui velocidade)
bool readAS5600(AS5600Data &out) {
  uint16_t cnt, raw;
  MagnetStatus st;
  uint8_t agc;
  uint16_t mag;

  if (!readAngleCounts(cnt)) return false;
  (void)readRawCounts(raw); // opcional: se falhar, mantemos 0
  if (!readStatus(st)) return false;
  (void)readAGC(agc);
  (void)readMagnitude(mag);

  uint16_t cntAdj = applyZero(cnt);

  // Tempo e velocidade
  unsigned long nowMs = millis();
  float velDeg_s = 0.0f, rpm = 0.0f;
  if (prevMs != 0) {
    float dt = (nowMs - prevMs) / 1000.0f;
    int16_t dCnt = diffWrapped(cntAdj, prevCnt);
    float cnt_s = dCnt / dt;
    velDeg_s = cnt_s * DEG_PER_CNT;
    rpm = cnt_s * (60.0f / CPR);
  }
  prevCnt = cntAdj;
  prevMs = nowMs;

  out.angleCnt  = cntAdj;
  out.rawCnt    = raw & 0x0FFF;
  out.angleDeg  = cntAdj * DEG_PER_CNT;
  out.angleRad  = cntAdj * RAD_PER_CNT;
  out.status    = st;
  out.agc       = agc;
  out.magnitude = mag;
  out.velDeg_s  = velDeg_s;
  out.rpm       = rpm;

  return true;
}

// Impressão amigável para debug/telemetria local
void printAS5600(const AS5600Data &d) {
  Serial.print(F("[AS5600] Ang(deg): "));
  Serial.print(d.angleDeg, 2);
  Serial.print(F(" | Vel(deg/s): "));
  Serial.print(d.velDeg_s, 2);
  Serial.print(F(" | RPM: "));
  Serial.print(d.rpm, 2);
  Serial.print(F(" | Cnt: "));
  Serial.print(d.angleCnt);
  Serial.print(F(" | RawCnt: "));
  Serial.print(d.rawCnt);

  Serial.print(F(" | Magnet: "));
  if (!d.status.magnetDetected) Serial.print(F("N/D"));
  else if (d.status.magnetTooWeak) Serial.print(F("Fraco"));
  else if (d.status.magnetTooStrong) Serial.print(F("Forte"));
  else Serial.print(F("OK"));

  Serial.print(F(" | AGC: "));
  Serial.print(d.agc);
  Serial.print(F(" | Magn: "));
  Serial.print(d.magnitude);

  Serial.println();
}

// =========================== Setup/Loop ===========================
void setup() {
  Serial.begin(115200);
  delay(100);
  Serial.println(F("\n=== AS5600 + ESP32 (I2C) - Exemplo Básico ==="));

  Wire.begin(I2C_SDA, I2C_SCL, I2C_FREQ);
  Serial.print(F("[Init] I2C em "));
  Serial.print(I2C_FREQ);
  Serial.println(F(" Hz"));

  // (Opcional) Aplique zero automático no boot:
  // setZeroToCurrent();
}

void loop() {
  AS5600Data data;
  if (readAS5600(data)) {
    if (millis() - lastPrintMs >= PRINT_INTERVAL_MS) {
      lastPrintMs = millis();
      printAS5600(data);
    }
    // Aqui você pode enviar 'data' por CAN/UART, gravar em SD, etc.
  } else {
    Serial.println(F("[AS5600] Falha na leitura (I2C). Verifique fiação/endereço."));
    delay(100);
  }

  // Exemplo: se quiser zerar ao apertar um botão, coloque a checagem aqui e chame setZeroToCurrent();
}
