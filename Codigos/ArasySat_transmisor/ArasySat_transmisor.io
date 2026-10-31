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
#define GPS_MIN_SATS 5        // minimo de satelites para usar la altitud GPS en la fusion
#define GPS_MAX_HDOP 2.5      // HDOP maximo aceptado (mas alto = geometria mala, se ignora)
#define GPS_SIGMA_BASE 5.0    // ruido tipico de la altitud GPS (m) con HDOP = 1; se escala con el HDOP
#define SIGMA_ACEL_BASE 0.3   // ruido base del acelerometro vertical (m/s^2); se suma 9.8 * vibracion medida

// ---------------- Determinador de fases de vuelo ----------------
// OJO: los umbrales de vibracion dependen del dron y deben calibrarse con un vuelo real (columna vibG del CSV)
#define VIB_TAU          1.0    // s: ventana con la que se mide la vibracion
#define FASE_V_ENTRA     0.6    // m/s: velocidad vertical que cuenta como ascenso o descenso
#define FASE_V_SALE      0.3    // m/s: por debajo (en valor absoluto) se considera sin movimiento vertical
#define FASE_T_MOV       1.0    // s: tiempo sostenido para confirmar ascenso / descenso
#define FASE_T_QUIETO    1.0    // s: tiempo sin movimiento vertical para confirmar estacionario (se suma a la ventana de 3 s)
#define FASE_T_SUELO     2.0    // s: tiempo quieto, cerca del suelo y con motores apagados para confirmar aterrizaje
#define FASE_T_SUELO_LARGO 10.0 // s: igual, pero sin exigir motores apagados (por si quedan en ralenti)
#define FASE_H_VUELO     3.0    // m sobre el suelo a partir de los cuales se considera que esta en el aire
#define FASE_H_SUELO     2.0    // m sobre el suelo por debajo de los cuales se considera tocando tierra
#define FASE_VIB_VUELO   0.04   // g: vibracion minima que indica motores encendidos
#define FASE_VIB_QUIETO  0.02   // g: vibracion maxima que indica motores apagados
#define FASE_VEL_GPS_MAX 3.0    // m/s: si el GPS indica mas velocidad horizontal que esto, no esta quieto en tierra
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
  "rollK,pitchK,altK,velVertK,fase,vibG";

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
float gpsAltFiltro = NAN;       // ultima altitud GPS capturada para la fusion
bool gpsAltNueva = false;       // hay una altitud GPS sin procesar
bool gpsAltValida = false;      // esa altitud paso las condiciones de calidad
float sigmaGpsAlt = 10.0;       // desviacion estimada de esa altitud (m)

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

// ---------------- Fusion de altitud: acelerometro + barometro + GPS ----------------
// Estados: [0] altura (m)  [1] velocidad vertical (m/s)  [2] bias del acelerometro (m/s^2)  [3] bias del barometro (m)
//  - El ACELEROMETRO predice el movimiento (a ~100 Hz).
//  - El BAROMETRO corrige a ~10 Hz: mide altura + bias_baro (muy preciso a corto plazo, referencia incierta).
//  - El GPS corrige a ~1 Hz: mide la altura absoluta (ruido de varios metros, pero sin offset ni deriva).
// Asi el barometro aporta la precision y el GPS fija la referencia absoluta estimando el bias del barometro.
// Usa double porque las covarianzas mezclan valores de cientos de m^2 con decimas y en float se degradan.
class KalmanAltura {
public:
  double sigma_acel = 0.5;    // ruido del acelerometro vertical (m/s^2). Sube si hay mucha vibracion
  double q_bias     = 1e-4;   // deriva del bias del acelerometro ((m/s^2)^2 por segundo)
  double q_bbaro    = 1e-3;   // deriva del bias del barometro (m^2 por segundo)
  double R_baro     = 0.25;   // varianza del barometro (m^2) -> 0.5 m de desviacion
  double sigma_b0   = 20.0;   // incertidumbre inicial de la referencia del barometro (m)

  void iniciar(double zBaro) {
    x[0] = zBaro; x[1] = 0; x[2] = 0; x[3] = 0;
    for (int i = 0; i < 4; i++) for (int j = 0; j < 4; j++) P[i][j] = 0;
    double sb2 = sigma_b0 * sigma_b0;
    // El barometro mide altura + bias, por eso ambos quedan correlacionados negativamente
    P[0][0] = sb2 + R_baro;  P[0][3] = -sb2;
    P[3][0] = -sb2;          P[3][3] = sb2;
    P[1][1] = 1.0;           P[2][2] = 1.0;
  }

  // aVert: aceleracion vertical SIN gravedad, positiva hacia arriba (m/s^2)
  void predecir(double aVert, double dt) {
    double a = aVert - x[2];
    x[0] += x[1] * dt + 0.5 * a * dt * dt;
    x[1] += a * dt;

    double dt2 = dt * dt;
    double F[4][4] = {{1, dt, -0.5 * dt2, 0}, {0, 1, -dt, 0}, {0, 0, 1, 0}, {0, 0, 0, 1}};
    double FP[4][4], Pn[4][4];
    for (int i = 0; i < 4; i++)
      for (int j = 0; j < 4; j++) {
        double acc = 0;
        for (int k = 0; k < 4; k++) acc += F[i][k] * P[k][j];
        FP[i][j] = acc;
      }
    for (int i = 0; i < 4; i++)
      for (int j = 0; j < 4; j++) {
        double acc = 0;
        for (int k = 0; k < 4; k++) acc += FP[i][k] * F[j][k];
        Pn[i][j] = acc;
      }

    double s2 = sigma_acel * sigma_acel;
    Pn[0][0] += s2 * 0.25 * dt2 * dt2;
    Pn[0][1] += s2 * 0.5 * dt2 * dt;
    Pn[1][0] += s2 * 0.5 * dt2 * dt;
    Pn[1][1] += s2 * dt2;
    Pn[2][2] += q_bias * dt;
    Pn[3][3] += q_bbaro * dt;

    for (int i = 0; i < 4; i++) for (int j = 0; j < 4; j++) P[i][j] = Pn[i][j];
  }

  void corregirBaro(double zBaro) {            // z = altura + bias_baro
    const double H[4] = {1, 0, 0, 1};
    actualizar(H, zBaro, R_baro);
  }

  void corregirGPS(double zGps, double sigma) { // z = altura absoluta
    const double H[4] = {1, 0, 0, 0};
    actualizar(H, zGps, sigma * sigma);
  }

  double alt()       { return x[0]; }
  double vel()       { return x[1]; }
  double biasAcel()  { return x[2]; }
  double biasBaro()  { return x[3]; }
  // Altura en la referencia del barometro: continua aunque el GPS corrija el offset (altura + bias_baro).
  // Es la que usa el determinador de fases para que un salto de referencia no parezca un movimiento.
  double altMarcoBaro() { return x[0] + x[3]; }
  double sigmaVel() { return sqrt(P[1][1]); }   // incertidumbre (1 sigma) de la velocidad vertical (m/s)

private:
  double x[4] = {0, 0, 0, 0};
  double P[4][4] = {{1, 0, 0, 0}, {0, 1, 0, 0}, {0, 0, 1, 0}, {0, 0, 0, 1}};

  void actualizar(const double H[4], double z, double R) {
    double PHt[4];
    for (int i = 0; i < 4; i++) {
      double acc = 0;
      for (int j = 0; j < 4; j++) acc += P[i][j] * H[j];
      PHt[i] = acc;
    }
    double S = R, y = z;
    for (int i = 0; i < 4; i++) { S += H[i] * PHt[i]; y -= H[i] * x[i]; }
    double K[4];
    for (int i = 0; i < 4; i++) K[i] = PHt[i] / S;
    for (int i = 0; i < 4; i++) x[i] += K[i] * y;
    for (int i = 0; i < 4; i++)
      for (int j = 0; j < 4; j++) P[i][j] -= K[i] * PHt[j];   // P es simetrica: (H P) = PHt'
    for (int i = 0; i < 4; i++)                                // mantiene la simetria
      for (int j = i + 1; j < 4; j++) {
        double m = 0.5 * (P[i][j] + P[j][i]);
        P[i][j] = m; P[j][i] = m;
      }
  }
};

KalmanAltura kalAltura;

// ---------------- Vibracion (desviacion tipica de la aceleracion total, en g) ----------------
class Vibracion {
public:
  float actualizar(float norma, float dt) {
    if (!listo) { media = norma; var = 0; listo = true; }
    float a = dt / VIB_TAU;
    if (a > 1.0f) a = 1.0f;
    float d = norma - media;
    media += a * d;
    var = (1.0f - a) * (var + a * d * d);
    return sqrt(var);
  }
private:
  float media = 1.0f, var = 0;
  bool listo = false;
};

// ---------------- Fases de vuelo: maquina de estados con histeresis ----------------
// 0 TIERRA -> 1 ASCENSO -> 2 ESTACIONARIO -> 3 DESCENSO -> 4 ATERRIZADO
enum FaseVuelo { F_TIERRA = 0, F_ASCENSO = 1, F_ESTACIONARIO = 2, F_DESCENSO = 3, F_ATERRIZADO = 4 };
const char* NOMBRE_FASE[] = {"TIERRA", "ASCENSO", "ESTACIONARIO", "DESCENSO", "ATERRIZADO"};
const char* CODIGO_FASE[] = {"TIE", "ASC", "EST", "DES", "ATE"};

class FasesVuelo {
public:
  int   fase = F_TIERRA;
  float hRel = 0;            // altura sobre el suelo (m)

  // dt: s | hAbs: altura fusionada en la referencia del barometro (m) | v: velocidad vertical fusionada (m/s)
  // sigmaV: incertidumbre de esa velocidad segun el filtro (m/s) | vib: vibracion del acelerometro (g) | velGps: velocidad horizontal del GPS (m/s, NAN si no hay dato)
  int actualizar(float dt, float hAbs, float v, float sigmaV, float vib, float velGps) {
    // --- Referencia de suelo: la altura de hace ~3 s. Se congela al despegar y se retoma al aterrizar ---
    if (!iniciado) {
      for (int i = 0; i < N_BUF; i++) buf[i] = hAbs;
      suelo = hAbs;
      iniciado = true;
    }
    tBuf += dt;
    if (tBuf >= 0.1f) {
      tBuf = 0;
      buf[idx] = hAbs;
      idx = (idx + 1) % N_BUF;           // buf[idx] pasa a ser la muestra mas antigua
    }
    if (fase == F_TIERRA || fase == F_ATERRIZADO) suelo = buf[idx];
    hRel = hAbs - suelo;

    // --- Condiciones instantaneas ---
    // Los umbrales crecen si el propio filtro dice que la velocidad es poco fiable (mucha vibracion).
    float umbralMov    = fmax(FASE_V_ENTRA, 3.0f * sigmaV);
    float umbralQuieto = fmax(FASE_V_SALE,  2.0f * sigmaV);
    // Velocidad media de los ultimos 3 s medida con la altura fusionada: mucho menos ruidosa que v
    float vVentana = (hAbs - buf[idx]) / (N_BUF * 0.1f);
    bool sube       = v >  umbralMov;
    bool baja       = v < -umbralMov;
    bool quieto     = fabs(v) < umbralQuieto && fabs(vVentana) < FASE_V_SALE;
    bool gpsQuieto  = isnan(velGps) || velGps < FASE_VEL_GPS_MAX;
    bool cercaSuelo = quieto && hRel < FASE_H_SUELO && gpsQuieto;

    // --- Temporizadores: cuanto tiempo seguido se cumple cada condicion ---
    tSube       = sube ? tSube + dt : 0;
    tBaja       = baja ? tBaja + dt : 0;
    tQuieto     = quieto ? tQuieto + dt : 0;
    tSuelo      = (cercaSuelo && vib < FASE_VIB_QUIETO) ? tSuelo + dt : 0;
    tSueloLargo = cercaSuelo ? tSueloLargo + dt : 0;

    // --- Transiciones ---
    switch (fase) {
      case F_TIERRA:
      case F_ATERRIZADO:
        // Despegue: sube de forma sostenida Y los motores estan encendidos (descarta un ascensor o una mano)
        if (tSube >= FASE_T_MOV && vib >= FASE_VIB_VUELO) fase = F_ASCENSO;
        break;

      case F_ASCENSO:
        if (tQuieto >= FASE_T_QUIETO && hRel > FASE_H_VUELO) fase = F_ESTACIONARIO;
        else if (tBaja >= FASE_T_MOV) fase = F_DESCENSO;
        else if (tQuieto >= FASE_T_SUELO && hRel < FASE_H_VUELO) fase = F_TIERRA;   // falso despegue
        break;

      case F_ESTACIONARIO:
        if (tSube >= FASE_T_MOV) fase = F_ASCENSO;
        else if (tBaja >= FASE_T_MOV) fase = F_DESCENSO;
        break;

      case F_DESCENSO:
        if (tSuelo >= FASE_T_SUELO || tSueloLargo >= FASE_T_SUELO_LARGO) fase = F_ATERRIZADO;
        else if (tQuieto >= FASE_T_QUIETO && hRel > FASE_H_VUELO) fase = F_ESTACIONARIO;
        else if (tSube >= FASE_T_MOV) fase = F_ASCENSO;
        break;
    }
    return fase;
  }

private:
  static const int N_BUF = 30;           // 30 muestras a 10 Hz = 3 s de historia
  float buf[N_BUF];
  int idx = 0;
  float tBuf = 0, suelo = 0;
  bool iniciado = false;
  float tSube = 0, tBaja = 0, tQuieto = 0, tSuelo = 0, tSueloLargo = 0;
};

Vibracion vibracion;
FasesVuelo fases;
int faseActual = F_TIERRA;
float vibG = 0;                          // vibracion actual (g)

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
  vibG = vibracion.actualizar(norma, dt);
  kalAltura.sigma_acel = SIGMA_ACEL_BASE + 9.80665 * vibG;   // mas vibracion = acelerometro menos confiable

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
  if (baroNueva) kalAltura.corregirBaro(altBaro);
  if (gpsAltNueva) {                      // altitud GPS nueva (~1 Hz): solo se fusiona si es confiable
    gpsAltNueva = false;
    if (gpsAltValida) kalAltura.corregirGPS(gpsAltFiltro, sigmaGpsAlt);
  }
  altK = kalAltura.alt();
  velK = kalAltura.vel();

  // ---- Fase de vuelo (usa la altura y la velocidad vertical ya fusionadas) ----
  float velGps = (gps.speed.isValid() && gps.speed.age() < 2000) ? gps.speed.mps() : NAN;
  int nueva = fases.actualizar(dt, (float)kalAltura.altMarcoBaro(), velK, (float)kalAltura.sigmaVel(), vibG, velGps);
  if (nueva != faseActual) {
    Serial.printf("FASE: %s -> %s  (h=%.1f m, v=%.2f m/s, vib=%.3f g)\n",
                  NOMBRE_FASE[faseActual], NOMBRE_FASE[nueva], fases.hRel, velK, vibG);
    faseActual = nueva;
  }
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
  display.print("P:"); display.print(pres, 1); display.print("hPa F:"); display.println(CODIGO_FASE[faseActual]);
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
  while (gpsSerial.available()) gps.encode(gpsSerial.read());

  // Altitud GPS para la fusion: se captura aqui apenas llega una trama nueva (antes de que
  // otra lectura de gps.altitude.meters() limpie el indicador isUpdated)
  if (gps.altitude.isUpdated()) {
    gpsAltFiltro = gps.altitude.meters();
    int satsFix = gps.satellites.isValid() ? gps.satellites.value() : 0;
    float hdop  = gps.hdop.isValid() ? gps.hdop.hdop() : 99.0;
    gpsAltValida = gps.altitude.isValid() && satsFix >= GPS_MIN_SATS && hdop <= GPS_MAX_HDOP;
    sigmaGpsAlt  = GPS_SIGMA_BASE * (hdop > 1.0 ? hdop : 1.0);
    gpsAltNueva  = true;
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
  // FASE (2): fase (0 TIERRA,1 ASCENSO,2 ESTACIONARIO,3 DESCENSO,4 ATERRIZADO), vibracion (g)
  String m = String(contador) + ",";
  m += f(ax, 3) + "," + f(ay, 3) + "," + f(az, 3) + ",";   // 3 decimales: antes 2 (resolucion de 0.01 g era del orden del ruido)
  m += f(gx, 2) + "," + f(gy, 2) + "," + f(gz, 2) + ",";   // 2 decimales: antes 1
  m += f(mx, 1) + "," + f(my, 1) + "," + f(mz, 1) + ",";
  m += f(tAHT, 2) + "," + f(hum, 1) + "," + f(tBMP, 2) + ",";
  m += f(pres, 2) + "," + f(alt, 2) + ",";             // presion con 2 decimales: antes 1 (0.1 hPa ~ 0.85 m)
  m += f(lat, 8) + "," + f(lon, 8) + "," + f(gAlt, 1) + ",";
  m += String(sats) + "," + f(vel, 1);
  m += "," + f(roll, 2) + "," + f(pitch, 2) + "," + f(altK, 2) + "," + f(velK, 3);
  m += "," + String(faseActual) + "," + f(vibG, 3);

  if (LoRa.beginPacket()) {        // devuelve 0 si la radio sigue transmitiendo
    LoRa.print(m);
    LoRa.endPacket(true);          // true = asincrono: el filtro sigue corriendo mientras se transmite
  } else {
    Serial.println("LoRa ocupado: paquete omitido");
  }

  guardarSD(m);

  Serial.println("Enviado (" + String(m.length()) + " bytes): " + m);
  mostrarPantalla(tBMP, hum, pres);
  Serial.println(gps.charsProcessed());   // depuracion del GPS: caracteres NMEA recibidos (si no crece, no llegan datos)

  contador++;
}
