/*

  GPS u-blox NEO-M8N + ESP32 (UART) - Exemplo básico e genérico
  - Usa TinyGPSPlus para parsear NMEA (lat/lon, alt, satélites, velocidade, data/hora, HDOP).
  - Pensado para integrar em projetos de telemetria/registro de dados.

  Wiring sugerido (ESP32 DevKit):
    GPS TX  -> ESP32 RX (GPIO16)  [GPS envia, ESP32 recebe]
    GPS RX  -> ESP32 TX (GPIO17)  [opcional: para enviar configs UBX]
    VCC     -> 3V3 (ou 5V conforme o seu módulo)
    GND     -> GND

  Dicas:
    - Taxa padrão de muitos M8N é 9600 bps em NMEA. Se ver "lixo" no Serial Monitor,
      ajuste o BAUD_GPS para 38400 ou 115200 conforme o seu módulo.
    - Abra o Serial Monitor em 115200 bps para ver os dados.


*/



#include <TinyGPSPlus.h>
#include <HardwareSerial.h>

// ===================== Configurações de pinos e seriais =====================
// UART usada para o GPS. ESP32 tem 3: 0 (Serial), 1 (Serial1), 2 (Serial2).
// Usaremos a UART2 para poder mapear os pinos facilmente.

static const int GPS_RX_PIN = 16; // ESP32 RX (conecte ao TX do GPS)
static const int GPS_TX_PIN = 17; // ESP32 TX (conecte ao RX do GPS) - opcional
static const int UART_NUM    = 2; // 2 => Serial2

// Baudrates
static const uint32_t BAUD_PC  = 115200; // Serial do PC (debug)
static const uint32_t BAUD_GPS = 9600;   // NMEA padrão da maioria dos M8N

// ========================= Objetos globais =========================
HardwareSerial gpsSerial(UART_NUM);
TinyGPSPlus gps;

// Estrutura de dados "genérica" para integrar no seu projeto
struct GPSData {
  bool   fix;           // true se temos um fix válido
  double lat;           // graus
  double lon;           // graus
  float  alt_m;         // altitude em metros
  float  speed_kmh;     // velocidade em km/h
  float  course_deg;    // rumo (0-360°), 0° = norte
  uint8_t sats;         // número de satélites
  float  hdop;          // precisão horizontal (quanto menor, melhor)
  uint16_t year;        // UTC
  uint8_t month, day;   // UTC
  uint8_t hour, minute, second; // UTC
};

// Controle de tempo para logs
unsigned long lastPrintMs = 0;
const unsigned long PRINT_INTERVAL_MS = 500; // imprime a cada 0,5 s (ajuste como quiser)

// ========================= Funções auxiliares =========================

// Alimenta o parser com os bytes vindos do GPS.
// Retorna true se pelo menos um caractere foi lido.
bool feedGPS() {
  bool readSomething = false;
  while (gpsSerial.available()) {
    char c = (char)gpsSerial.read();
    readSomething = true;
    gps.encode(c); // TinyGPSPlus processa o byte
  }
  return readSomething;
}

// Preenche a struct com os dados atuais do TinyGPSPlus
void getGPSData(GPSData &out) {
  out.fix = gps.location.isValid() && gps.location.isUpdated();

  out.lat = gps.location.isValid() ? gps.location.lat() : NAN;
  out.lon = gps.location.isValid() ? gps.location.lng() : NAN;
  out.alt_m = gps.altitude.isValid() ? gps.altitude.meters() : NAN;

  out.speed_kmh = gps.speed.isValid() ? gps.speed.kmph() : NAN;
  out.course_deg = gps.course.isValid() ? gps.course.deg() : NAN;

  out.sats = gps.satellites.isValid() ? (uint8_t)gps.satellites.value() : 0;
  out.hdop = gps.hdop.isValid() ? gps.hdop.hdop() : NAN;

  out.year = gps.date.isValid() ? gps.date.year() : 0;
  out.month = gps.date.isValid() ? gps.date.month() : 0;
  out.day = gps.date.isValid() ? gps.date.day() : 0;

  out.hour = gps.time.isValid() ? gps.time.hour() : 0;
  out.minute = gps.time.isValid() ? gps.time.minute() : 0;
  out.second = gps.time.isValid() ? gps.time.second() : 0;
}

// Impressão legível para debug/telemetria local
void printGPS(const GPSData &d) {
  Serial.print(F("[GPS] Fix: "));
  Serial.print(d.fix ? F("SIM") : F("NAO"));
  Serial.print(F(" | Lat: "));     Serial.print(d.lat, 6);
  Serial.print(F(" | Lon: "));     Serial.print(d.lon, 6);
  Serial.print(F(" | Alt(m): "));  Serial.print(d.alt_m, 1);
  Serial.print(F(" | Vel(km/h): ")); Serial.print(d.speed_kmh, 1);
  Serial.print(F(" | Rumo(°): ")); Serial.print(d.course_deg, 1);
  Serial.print(F(" | Sats: "));    Serial.print(d.sats);
  Serial.print(F(" | HDOP: "));    Serial.print(d.hdop, 1);

  Serial.print(F(" | UTC "));
  if (d.year) {
    // Formata data/hora UTC se válida
    if (d.day < 10) Serial.print('0');
    Serial.print(d.day); Serial.print('/');
    if (d.month < 10) Serial.print('0');
    Serial.print(d.month); Serial.print('/');
    Serial.print(d.year); Serial.print(' ');
    if (d.hour < 10) Serial.print('0');
    Serial.print(d.hour); Serial.print(':');
    if (d.minute < 10) Serial.print('0');
    Serial.print(d.minute); Serial.print(':');
    if (d.second < 10) Serial.print('0');
    Serial.print(d.second);
  } else {
    Serial.print(F(" --/--/---- --:--:--"));
  }

  Serial.println();
}

// (Opcional) Checagem de travamento: se não chega nada do GPS por X ms, avisa
void watchdogGPS(unsigned long noDataTimeoutMs = 3000) {
  static unsigned long lastByteMs = 0;
  static bool initialized = false;

  // Atualiza timestamp sempre que chegar algo
  if (gpsSerial.available()) {
    // Consome, mas quem parseia mesmo é a feedGPS() no loop
    // Aqui só usamos para detectar "vida"
    while (gpsSerial.available()) {
      (void)gpsSerial.read();
      lastByteMs = millis();
    }
    if (!initialized) initialized = true;
  }

  if (initialized && (millis() - lastByteMs > noDataTimeoutMs)) {
    Serial.println(F("[GPS] Aviso: nenhum dado recebido do módulo há alguns segundos. Verifique fiação/baudrate/alimentação."));
    // Reinicia o marcador para não spammar
    lastByteMs = millis();
  }
}

// ============================== Setup/Loop ==============================
void setup() {
  Serial.begin(BAUD_PC);
  delay(100);
  Serial.println(F("\n=== GPS u-blox NEO-M8N + ESP32 - Exemplo ==="));

  // Inicia UART2 mapeando pinos RX/TX desejados
  gpsSerial.begin(BAUD_GPS, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
  Serial.print(F("[Init] UART2 em ")); Serial.print(BAUD_GPS);
  Serial.print(F(" bps | RX=")); Serial.print(GPS_RX_PIN);
  Serial.print(F(" TX=")); Serial.println(GPS_TX_PIN);

  // (Opcional) Aqui você pode enviar comandos UBX para ajustar taxa/sentencas.
  // Mantemos o exemplo básico somente lendo NMEA padrão do módulo.
}

void loop() {
  // 1) Ler e alimentar o parser
  feedGPS();

  // 2) A cada PRINT_INTERVAL_MS, capturar snapshot e imprimir
  if (millis() - lastPrintMs >= PRINT_INTERVAL_MS) {
    lastPrintMs = millis();
    GPSData data;
    getGPSData(data);
    printGPS(data);
  }

  // 3) (Opcional) watchdog para diagnosticar ausência de dados
  // watchdogGPS();
}

/* ====================== Extras/Notas para o seu projeto ======================
  - Integração: a função getGPSData() enche a struct GPSData. É fácil jogar isso
    em um buffer, mandar por CAN/UART para outro módulo, gravar em SD, etc.

  - Fix vs validade: TinyGPSPlus tem "isValid()" (já viu dado válido) e "isUpdated()"
    (acabou de atualizar). Aqui marcamos fix = válido e atualizado; ajuste conforme
    sua lógica (por exemplo, aceitar válido mesmo sem "updated" no ciclo atual).

  - Taxa de atualização (Hz): M8N vem geralmente em 1 Hz. Para 5–10 Hz com menos
    carga na UART, costuma-se desativar sentenças NMEA que não usa (ex.: deixar
    só GGA e RMC) e aumentar a taxa via UBX-CFG-RATE. Se quiser, eu te mando uma
    função pronta para enviar os pacotes UBX corretos (com checksum).

  - Fuso horário: GPS fornece tempo *UTC*. Para hora local no Brasil, aplique o
    offset do seu fuso (e regras de horário de verão, quando existir).

  - Alimentação: garanta GND comum, fonte estável e antena com boa visada do céu.
    Em ambiente interno/garagem, o fix pode demorar ou nem acontecer.
*/
