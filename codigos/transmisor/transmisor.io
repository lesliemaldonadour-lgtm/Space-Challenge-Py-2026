#include <SPI.h>
#include <LoRa.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_AHTX0.h>     // AHT20
#include <Adafruit_BMP280.h>    // BMP280
#include <TinyGPSPlus.h>        // GPS NEO-6M
#include <FS.h>
#include <SD.h>                  // Lector microSD de la placa (bus SPI propio)

// ---------------- Pines LoRa del TTGO LoRa32 V2.1 (T3 V1.6.1) ----------------
#define LORA_SCK   5
#define LORA_MISO  19
#define LORA_MOSI  27
#define LORA_CS    18
#define LORA_RST   23
#define LORA_DIO0  26

// ---------------- microSD integrada del TTGO LoRa32 V2.1 (T3 V1.6.1) ----------------
// Usa su propio bus SPI (HSPI), asi no choca con el LoRa (VSPI)
#define SD_SCK     14
#define SD_MISO    2
#define SD_MOSI    15
#define SD_CS      13
#define SD_FLUSH_MS    1000   // cada cuanto se fuerza el guardado fisico en la tarjeta
#define SD_REINTENTO_MS 5000  // si la SD falla, reintenta iniciarla cada 5 s

// ---------------- Bus I2C (compartido: OLED + todos los sensores) ----------------
#define I2C_SDA    21
#define I2C_SCL    22
#define OLED_RST   -1
#define ANCHO      128
#define ALTO       64

// ---------------- GPS (UART2) ----------------
#define GPS_RX_PIN 34   // Pin del ESP32 que recibe  -> va al TX del GPS
#define GPS_TX_PIN 4    // Pin del ESP32 que envia   -> va al RX del GPS (opcional)
#define GPS_BAUD   9600

#define FRECUENCIA 433E6      // 433 MHz
#define INTERVALO_ENVIO 1000  // ms entre paquetes (antes 2000)

// ---------------- Direcciones I2C ----------------
#define MPU_ADDR   0x68   // GY-521 (MPU6050)
#define HMC_ADDR   0x1E   // GY-273 si el chip es HMC5883L
#define QMC_ADDR   0x0D   // GY-273 si el chip es QMC5883L (muy comun en clones)
#define SEA_LEVEL_HPA 1013.25

// ---------------- Filtro de Kalman ----------------
#define PERIODO_MPU_US 10000  // el MPU se lee y filtra cada 10 ms (~100 Hz), independiente del envio LoRa
#define PERIODO_BARO_MS 100   // el barometro se lee para el filtro de altitud cada 100 ms (~10 Hz)
#define MPU_DLPF 0            // filtro interno del MPU: 0 = apagado (datos crudos). Con mucha vibracion prueba 3 (44 Hz) o 4 (21 Hz)

Adafruit_SSD1306 display(ANCHO, ALTO, &Wire, OLED_RST);
Adafruit_AHTX0 aht;
Adafruit_BMP280 bmp(&Wire);
TinyGPSPlus gps;
HardwareSerial gpsSerial(2);
SPIClass sdSPI(HSPI);
File archivoLog;
bool okSD = false;
char nombreLog[24] = "";
unsigned long ultimoFlushSD = 0, ultimoReintentoSD = 0;
const char CABECERA_CSV[] =
  "ms,N,ax,ay,az,gx,gy,gz,mx,my,mz,tAHT,hum,tBMP,pres,alt,lat,lon,gpsAlt,sats,vel,"
  "rollK,pitchK,altK,velVertK";

bool okMPU = false, okMAG = false, okAHT = false, okBMP = false;
uint8_t magAddr = 0;
unsigned long ultimoEnvio = 0;
int contador = 0;

// Ultima lectura del MPU (se actualiza a ~100 Hz) y angulos filtrados
float ax = NAN, ay = NAN, az = NAN, gx = NAN, gy = NAN, gz = NAN;
float roll = NAN, pitch = NAN;
unsigned long ultimoMPUus = 0;
bool kalmanListo = false;
float altBaro = NAN;            // ultima lectura cruda del barometro (para el filtro)
float altK = NAN, velK = NAN;   // altitud (m) y velocidad vertical (m/s) filtradas
unsigned long ultimoBaroMs = 0;
bool alturaLista = false;

// ---------------- Utilidades I2C ----------------
bool i2cPresente(uint8_t addr) {
  Wire.beginTransmission(addr);
  return Wire.endTransmission() == 0;
}

void i2cEscribir(uint8_t addr, uint8_t reg, uint8_t val) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
}

bool i2cLeer(uint8_t addr, uint8_t reg, uint8_t *buf, uint8_t len) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  if ((int)Wire.requestFrom((int)addr, (int)len) != len) return false;
  for (uint8_t i = 0; i < len; i++) buf[i] = Wire.read();
  return true;
}

String f(float v, int dec) {
  return isnan(v) ? String("nan") : String(v, dec);
}

// ---------------- GY-521 (MPU6050) ----------------
void iniciarMPU() {
  if (!i2cPresente(MPU_ADDR)) return;
  i2cEscribir(MPU_ADDR, 0x6B, 0x00);   // despertar
  i2cEscribir(MPU_ADDR, 0x1A, MPU_DLPF); // filtro pasa-bajos interno (0 = apagado)
  i2cEscribir(MPU_ADDR, 0x1C, 0x08);   // acelerometro +-4g
  i2cEscribir(MPU_ADDR, 0x1B, 0x08);   // giroscopio +-500 dps
  okMPU = true;
}

void leerMPU(float &ax, float &ay, float &az, float &gx, float &gy, float &gz) {
  ax = ay = az = gx = gy = gz = NAN;
  if (!okMPU) return;
  uint8_t b[14];
  if (!i2cLeer(MPU_ADDR, 0x3B, b, 14)) return;
  int16_t rax = (b[0] << 8) | b[1];
  int16_t ray = (b[2] << 8) | b[3];
  int16_t raz = (b[4] << 8) | b[5];
  int16_t rgx = (b[8] << 8) | b[9];
  int16_t rgy = (b[10] << 8) | b[11];
  int16_t rgz = (b[12] << 8) | b[13];
  ax = rax / 8192.0; ay = ray / 8192.0; az = raz / 8192.0;   // g     (+-4 g  -> 8192 LSB/g)
  gx = rgx / 65.5;   gy = rgy / 65.5;   gz = rgz / 65.5;     // dps   (+-500 dps -> 65.5 LSB/dps)
}

// ---------------- GY-273 (HMC5883L / QMC5883L) ----------------
void iniciarMag() {
  if (i2cPresente(HMC_ADDR)) {
    magAddr = HMC_ADDR;
    i2cEscribir(HMC_ADDR, 0x00, 0x70);  // 8 muestras, 15 Hz
    i2cEscribir(HMC_ADDR, 0x01, 0x20);  // ganancia +-1.3 Ga
    i2cEscribir(HMC_ADDR, 0x02, 0x00);  // modo continuo
    okMAG = true;
  } else if (i2cPresente(QMC_ADDR)) {
    magAddr = QMC_ADDR;
    i2cEscribir(QMC_ADDR, 0x0B, 0x01);  // periodo set/reset
    i2cEscribir(QMC_ADDR, 0x09, 0x1D);  // continuo, 200 Hz, 8 G, OSR 512
    okMAG = true;
  }
}

void leerMag(float &mx, float &my, float &mz) {   // en microteslas (uT)
  mx = my = mz = NAN;
  if (!okMAG) return;
  uint8_t b[6];
  if (magAddr == HMC_ADDR) {
    if (!i2cLeer(HMC_ADDR, 0x03, b, 6)) return;
    int16_t x = (b[0] << 8) | b[1];
    int16_t z = (b[2] << 8) | b[3];
    int16_t y = (b[4] << 8) | b[5];
    mx = x * 0.092; my = y * 0.092; mz = z * 0.092;
  } else {
    if (!i2cLeer(QMC_ADDR, 0x00, b, 6)) return;
    int16_t x = (b[1] << 8) | b[0];
    int16_t y = (b[3] << 8) | b[2];
    int16_t z = (b[5] << 8) | b[4];
    mx = x / 30.0; my = y / 30.0; mz = z / 30.0;
  }
}

// ---------------- Filtro de Kalman (2 estados: angulo + bias del giroscopio) ----------------
class KalmanAngulo {
public:
  float Q_angulo = 0.001f;   // ruido del proceso: angulo
  float Q_bias   = 0.003f;   // ruido del proceso: bias del giroscopio
  float R_medida = 0.03f;    // ruido de medida del acelerometro (grados^2)

  void iniciar(float anguloInicial) {
    angulo = anguloInicial;
    bias = 0;
    P[0][0] = 1; P[0][1] = 0;
    P[1][0] = 0; P[1][1] = 1;
  }

  // anguloAcc: angulo del acelerometro (deg), velGiro: giroscopio (deg/s), dt: segundos
  // usarAcc=false -> solo prediccion con el giroscopio (acelerometro no confiable)
  float actualizar(float anguloAcc, float velGiro, float dt, bool usarAcc) {
    // --- Prediccion ---
    float tasa = velGiro - bias;
    angulo += dt * tasa;
    P[0][0] += dt * (dt * P[1][1] - P[0][1] - P[1][0] + Q_angulo);
    P[0][1] -= dt * P[1][1];
    P[1][0] -= dt * P[1][1];
    P[1][1] += Q_bias * dt;

    // --- Correccion ---
    if (usarAcc) {
      float S  = P[0][0] + R_medida;
      float K0 = P[0][0] / S;
      float K1 = P[1][0] / S;
      float y  = envolver(anguloAcc - angulo);   // innovacion
      angulo += K0 * y;
      bias   += K1 * y;
      float p00 = P[0][0], p01 = P[0][1];
      P[0][0] -= K0 * p00;  P[0][1] -= K0 * p01;
      P[1][0] -= K1 * p00;  P[1][1] -= K1 * p01;
    }
    angulo = envolver(angulo);
    return angulo;
  }

  float getBias() { return bias; }

private:
  float angulo = 0, bias = 0;
  float P[2][2] = {{1, 0}, {0, 1}};
  static float envolver(float a) {      // lleva el angulo a [-180, 180]
    while (a > 180.0f)  a -= 360.0f;
    while (a < -180.0f) a += 360.0f;
    return a;
  }
};

KalmanAngulo kalRoll, kalPitch;

// ---------------- Kalman de altitud (3 estados: altura, velocidad vertical, bias del acelerometro) ----------------
// Predice con la aceleracion vertical del acelerometro y corrige con el barometro.
class KalmanAltura {
public:
  float sigma_acel = 0.5f;    // ruido del acelerometro vertical (m/s^2). Sube si hay mucha vibracion
  float q_bias     = 1e-4f;   // deriva del bias del acelerometro ((m/s^2)^2 por segundo)
  float R_baro     = 0.25f;   // varianza del barometro (m^2) -> 0.5 m de desviacion

  void iniciar(float alt0) {
    h = alt0; v = 0; b = 0;
    for (int i = 0; i < 3; i++) for (int j = 0; j < 3; j++) P[i][j] = 0;
    P[0][0] = 1.0f; P[1][1] = 1.0f; P[2][2] = 1.0f;
  }

  // aVert: aceleracion vertical SIN gravedad, positiva hacia arriba (m/s^2)
  void predecir(float aVert, float dt) {
    float a = aVert - b;
    h += v * dt + 0.5f * a * dt * dt;
    v += a * dt;

    float dt2 = dt * dt;
    float F[3][3] = {{1, dt, -0.5f * dt2}, {0, 1, -dt}, {0, 0, 1}};
    float FP[3][3], Pn[3][3];
    for (int i = 0; i < 3; i++)
      for (int j = 0; j < 3; j++)
        FP[i][j] = F[i][0] * P[0][j] + F[i][1] * P[1][j] + F[i][2] * P[2][j];
    for (int i = 0; i < 3; i++)
      for (int j = 0; j < 3; j++)
        Pn[i][j] = FP[i][0] * F[j][0] + FP[i][1] * F[j][1] + FP[i][2] * F[j][2];

    float s2 = sigma_acel * sigma_acel;
    Pn[0][0] += s2 * 0.25f * dt2 * dt2;
    Pn[0][1] += s2 * 0.5f * dt2 * dt;
    Pn[1][0] += s2 * 0.5f * dt2 * dt;
    Pn[1][1] += s2 * dt2;
    Pn[2][2] += q_bias * dt;

    for (int i = 0; i < 3; i++) for (int j = 0; j < 3; j++) P[i][j] = Pn[i][j];
  }

  void corregir(float altBaro) {
    float S = P[0][0] + R_baro;
    float K[3] = {P[0][0] / S, P[1][0] / S, P[2][0] / S};
    float y = altBaro - h;
    h += K[0] * y;
    v += K[1] * y;
    b += K[2] * y;
    float p0[3] = {P[0][0], P[0][1], P[0][2]};
    for (int i = 0; i < 3; i++)
      for (int j = 0; j < 3; j++)
        P[i][j] -= K[i] * p0[j];
  }

  float alt() { return h; }
  float vel() { return v; }
  float biasAcel() { return b; }

private:
  float h = 0, v = 0, b = 0;
  float P[3][3] = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
};

KalmanAltura kalAltura;

// Lee el MPU y el barometro y actualiza los filtros. Se llama en cada vuelta del loop.
void actualizarKalman() {
  unsigned long ahora = micros();
  unsigned long dtUs = ahora - ultimoMPUus;
  if (dtUs < PERIODO_MPU_US) return;
  ultimoMPUus = ahora;
  float dt = dtUs / 1e6f;

  leerMPU(ax, ay, az, gx, gy, gz);
  if (isnan(ax)) return;   // lectura fallida: se conserva el ultimo estado

  // ---- Roll y pitch (mismo calculo de angulos por acelerometro que el codigo de referencia) ----
  float accRoll  = atan2(ay, az) * 180.0 / PI;
  float accPitch = atan2(-ax, sqrt(ay * ay + az * az)) * 180.0 / PI;

  if (!kalmanListo) {                 // primera lectura: arranca desde el angulo del acelerometro
    kalRoll.iniciar(accRoll);
    kalPitch.iniciar(accPitch);
    roll = accRoll;
    pitch = accPitch;
    kalmanListo = true;
    return;
  }

  // Si la aceleracion total se aleja de 1 g (caida libre, golpe), el acelerometro no sirve
  // para medir inclinacion: ese ciclo se usa solo el giroscopio.
  float norma = sqrt(ax * ax + ay * ay + az * az);
  bool accValido = (norma > 0.7 && norma < 1.3);

  roll  = kalRoll.actualizar(accRoll,  gx, dt, accValido);
  pitch = kalPitch.actualizar(accPitch, gy, dt, accValido);

  // ---- Altitud y velocidad vertical ----
  bool baroNueva = false;
  if (okBMP && millis() - ultimoBaroMs >= PERIODO_BARO_MS) {
    ultimoBaroMs = millis();
    altBaro = bmp.readAltitude(SEA_LEVEL_HPA);
    baroNueva = !isnan(altBaro);
  }

  if (!alturaLista) {                 // arranca con la primera lectura valida del barometro
    if (baroNueva) {
      kalAltura.iniciar(altBaro);
      alturaLista = true;
      altK = altBaro;
      velK = 0;
    }
    return;
  }

  // Aceleracion vertical en ejes del mundo (sin gravedad), usando roll/pitch ya filtrados
  float r = roll * PI / 180.0, pt = pitch * PI / 180.0;
  float azMundo = -ax * sin(pt) + ay * sin(r) * cos(pt) + az * cos(r) * cos(pt);   // g
  float aVert = (azMundo - 1.0) * 9.80665;                                          // m/s^2

  kalAltura.predecir(aVert, dt);
  if (baroNueva) kalAltura.corregir(altBaro);
  altK = kalAltura.alt();
  velK = kalAltura.vel();
}

// ---------------- microSD ----------------
// Crea un archivo nuevo (datos_000.csv, datos_001.csv, ...) en cada arranque para no sobrescribir vuelos anteriores.
bool iniciarSD() {
  if (archivoLog) archivoLog.close();
  SD.end();
  sdSPI.begin(SD_SCK, SD_MISO, SD_MOSI, SD_CS);
  if (!SD.begin(SD_CS, sdSPI)) {
    okSD = false;
    return false;
  }
  for (int i = 0; i < 1000; i++) {
    snprintf(nombreLog, sizeof(nombreLog), "/datos_%03d.csv", i);
    if (!SD.exists(nombreLog)) break;
  }
  archivoLog = SD.open(nombreLog, FILE_WRITE);
  if (!archivoLog) {
    okSD = false;
    return false;
  }
  archivoLog.println(CABECERA_CSV);
  archivoLog.flush();
  okSD = true;
  Serial.printf("SD OK -> %s\n", nombreLog);
  return true;
}

// Guarda una linea en la SD: millis() + el mismo CSV que se envia por LoRa
void guardarSD(const String &paquete) {
  if (!okSD) {
    // sin SD: reintenta de vez en cuando (por si la insertaron o fue un fallo momentaneo)
    if (millis() - ultimoReintentoSD >= SD_REINTENTO_MS) {
      ultimoReintentoSD = millis();
      iniciarSD();
    }
    return;
  }
  size_t n = archivoLog.print(millis());
  n += archivoLog.print(',');
  n += archivoLog.println(paquete);
  if (n == 0) {                       // la escritura fallo (tarjeta retirada, llena, etc.)
    Serial.println("Error escribiendo en SD");
    okSD = false;
    return;
  }
  if (millis() - ultimoFlushSD >= SD_FLUSH_MS) {
    ultimoFlushSD = millis();
    archivoLog.flush();               // sin flush, un corte de energia pierde los ultimos datos
  }
}

// ---------------- Pantalla ----------------
void mostrarPantalla(float temp, float hum, float pres) {
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(2);
  display.setCursor(0, 0);
  display.println("TRANSMISOR");
  display.setTextSize(1);
  display.setCursor(0, 20);
  display.print("Paquete #"); display.print(contador);
  display.println(okSD ? "  SD OK" : "  SD --");
  display.setCursor(0, 30);
  display.print("T:"); display.print(temp, 1);
  display.print("C H:"); display.print(hum, 0); display.println("%");
  display.setCursor(0, 40);
  display.print("P:"); display.print(pres, 1); display.println("hPa");
  display.setCursor(0, 50);
  display.print("GPS sats:"); display.print(gps.satellites.isValid() ? gps.satellites.value() : 0);
  display.print(gps.location.isValid() ? " FIX" : " --");
  display.display();
}

void setup() {
  Serial.begin(9600);

  Wire.begin(I2C_SDA, I2C_SCL);

  // OLED
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println("Error al iniciar OLED");
    while (true);
  }
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(2);
  display.setCursor(0, 0);
  display.println("TRANSMISOR");
  display.setTextSize(1);
  display.setCursor(0, 30);
  display.println("Iniciando...");
  display.display();

  // Sensores
  iniciarMPU();
  ultimoMPUus = micros();
  iniciarMag();
  okAHT = aht.begin();
  okBMP = bmp.begin(0x77) || bmp.begin(0x76);
  if (okBMP) {
    bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,
                    Adafruit_BMP280::SAMPLING_X2,
                    Adafruit_BMP280::SAMPLING_X4,
                    Adafruit_BMP280::FILTER_X4,        // antes X16: demasiado retardo al subir
                    Adafruit_BMP280::STANDBY_MS_63);   // antes 500 ms: ~13 lecturas/s
  }

  Serial.printf("MPU6050 (GY-521): %s\n", okMPU ? "OK" : "NO ENCONTRADO");
  Serial.printf("Magnetometro (GY-273): %s (0x%02X)\n", okMAG ? "OK" : "NO ENCONTRADO", magAddr);
  Serial.printf("AHT20: %s\n", okAHT ? "OK" : "NO ENCONTRADO");
  Serial.printf("BMP280: %s\n", okBMP ? "OK" : "NO ENCONTRADO");

  // microSD (si falla, el transmisor sigue funcionando solo por LoRa)
  iniciarSD();
  if (!okSD) Serial.println("SD: no se pudo iniciar (revisa tarjeta FAT32)");

  // GPS
  gpsSerial.setRxBufferSize(1024);   // antes de begin()
  gpsSerial.begin(GPS_BAUD, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);


  // LoRa
  SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_CS);
  LoRa.setPins(LORA_CS, LORA_RST, LORA_DIO0);

  if (!LoRa.begin(FRECUENCIA)) {
    Serial.println("Error al iniciar LoRa");
    display.clearDisplay();
    display.setTextSize(1);
    display.setCursor(0, 0);
    display.println("Error LoRa");
    display.display();
    while (true);
  }

  // Deben ser IGUALES en el receptor
  LoRa.setSpreadingFactor(7);
  LoRa.setSignalBandwidth(125E3);
  LoRa.setCodingRate4(5);
  LoRa.setSyncWord(0x12);

  Serial.println("Transmisor listo");
}

void loop() {
  // Alimentar al GPS continuamente (no usar delay largos)
  while (gpsSerial.available()) {
  gps.encode(gpsSerial.read());
}
  // MPU + Kalman a ~100 Hz (corre en cada vuelta, el envio LoRa sigue cada INTERVALO_ENVIO)
  actualizarKalman();

  if (millis() - ultimoEnvio < INTERVALO_ENVIO) return;
  ultimoEnvio = millis();

  // --- Leer sensores ---
  // ax..gz y roll/pitch ya vienen de actualizarKalman()
  float mx, my, mz;
  leerMag(mx, my, mz);

  float tAHT = NAN, hum = NAN, tBMP = NAN, pres = NAN, alt = NAN;
  if (okAHT) {
    sensors_event_t eHum, eTemp;
    aht.getEvent(&eHum, &eTemp);
    tAHT = eTemp.temperature;
    hum  = eHum.relative_humidity;
  }
  if (okBMP) {
    tBMP = bmp.readTemperature();
    pres = bmp.readPressure() / 100.0;       // hPa
    alt  = bmp.readAltitude(SEA_LEVEL_HPA);  // m
  }

  float lat = NAN, lon = NAN, gAlt = NAN, vel = NAN;
  int sats = gps.satellites.isValid() ? gps.satellites.value() : 0;
  if (gps.location.isValid()) { lat = gps.location.lat(); lon = gps.location.lng(); }
  if (gps.altitude.isValid()) gAlt = gps.altitude.meters();
  if (gps.speed.isValid())    vel  = gps.speed.kmph();

  // --- Armar paquete CSV ---
  // CRUDOS   (20): N,ax,ay,az,gx,gy,gz,mx,my,mz,tAHT,hum,tBMP,pres,alt,lat,lon,gpsAlt,sats,vel
  // FILTRADOS (4): rollKalman,pitchKalman,altKalman,velVertKalman
  String m = String(contador) + ",";
  m += f(ax, 2) + "," + f(ay, 2) + "," + f(az, 2) + ",";
  m += f(gx, 1) + "," + f(gy, 1) + "," + f(gz, 1) + ",";
  m += f(mx, 1) + "," + f(my, 1) + "," + f(mz, 1) + ",";
  m += f(tAHT, 1) + "," + f(hum, 1) + "," + f(tBMP, 1) + ",";
  m += f(pres, 1) + "," + f(alt, 1) + ",";
  m += f(lat, 8) + "," + f(lon, 8) + "," + f(gAlt, 1) + ",";
  m += String(sats) + "," + f(vel, 1);
  m += "," + f(roll, 2) + "," + f(pitch, 2) + "," + f(altK, 2) + "," + f(velK, 2);

  if (LoRa.beginPacket()) {        // devuelve 0 si la radio sigue transmitiendo
    LoRa.print(m);
    LoRa.endPacket(true);          // true = asincrono: el filtro sigue corriendo mientras se transmite
  } else {
    Serial.println("LoRa ocupado: paquete omitido");
  }

  guardarSD(m);

  Serial.println("Enviado (" + String(m.length()) + " bytes): " + m);
  mostrarPantalla(tBMP, hum, pres);
  Serial.println(gps.charsProcessed());
  contador++;
}
