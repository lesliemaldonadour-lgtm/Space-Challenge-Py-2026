#include <SPI.h>
#include <LoRa.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// LoRa (TTGO LoRa32 V1.6)
#define LORA_SCK  5
#define LORA_MISO 19
#define LORA_MOSI 27
#define LORA_CS   18
#define LORA_RST  23
#define LORA_DIO0 26
#define LORA_FREQ 433E6

#define I2C_SDA 21
#define I2C_SCL 22

#define ADDR_HMC 0x1E
#define ADDR_QMC 0x0D

// Declinación magnética en grados (ajústala a tu zona; 0 = sin corrección)
#define DECLINACION 0.0

Adafruit_SSD1306 display(128, 64, &Wire, -1);

uint8_t addrMag = 0;   // dirección detectada
int contador = 0;

bool existeI2C(uint8_t addr) {
  Wire.beginTransmission(addr);
  return Wire.endTransmission() == 0;
}

void escribirReg(uint8_t addr, uint8_t reg, uint8_t val) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
}

bool iniciarMagnetometro() {
  if (existeI2C(ADDR_HMC)) {
    addrMag = ADDR_HMC;
    escribirReg(ADDR_HMC, 0x00, 0x70);  // 8 muestras, 15 Hz
    escribirReg(ADDR_HMC, 0x01, 0x20);  // ganancia
    escribirReg(ADDR_HMC, 0x02, 0x00);  // modo continuo
    Serial.println("Magnetometro: HMC5883L");
    return true;
  }
  if (existeI2C(ADDR_QMC)) {
    addrMag = ADDR_QMC;
    escribirReg(ADDR_QMC, 0x0B, 0x01);  // periodo set/reset
    escribirReg(ADDR_QMC, 0x09, 0x1D);  // continuo, 200 Hz, 8G, OSR 512
    Serial.println("Magnetometro: QMC5883L");
    return true;
  }
  return false;
}

bool leerMagnetometro(int16_t &x, int16_t &y, int16_t &z) {
  uint8_t reg = (addrMag == ADDR_HMC) ? 0x03 : 0x00;

  Wire.beginTransmission(addrMag);
  Wire.write(reg);
  if (Wire.endTransmission() != 0) return false;

  if (Wire.requestFrom(addrMag, (uint8_t)6) != 6) return false;

  uint8_t b[6];
  for (int i = 0; i < 6; i++) b[i] = Wire.read();

  if (addrMag == ADDR_HMC) {
    // Orden: X, Z, Y (MSB primero)
    x = (int16_t)((b[0] << 8) | b[1]);
    z = (int16_t)((b[2] << 8) | b[3]);
    y = (int16_t)((b[4] << 8) | b[5]);
  } else {
    // Orden: X, Y, Z (LSB primero)
    x = (int16_t)((b[1] << 8) | b[0]);
    y = (int16_t)((b[3] << 8) | b[2]);
    z = (int16_t)((b[5] << 8) | b[4]);
  }
  return true;
}

void setup() {
  Serial.begin(115200);
  Wire.begin(I2C_SDA, I2C_SCL);

  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println("Error OLED");
    while (true);
  }
  display.setTextColor(WHITE);

  if (!iniciarMagnetometro()) {
    Serial.println("No se encontro el GY-273");
  }

  SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_CS);
  LoRa.setPins(LORA_CS, LORA_RST, LORA_DIO0);
  if (!LoRa.begin(LORA_FREQ)) {
    Serial.println("Error LoRa");
    while (true);
  }
  Serial.println("Transmisor listo");
}

void loop() {
  int16_t x = 0, y = 0, z = 0;
  float rumbo = NAN;

  if (addrMag != 0 && leerMagnetometro(x, y, z)) {
    rumbo = atan2((float)y, (float)x) * 180.0 / PI + DECLINACION;
    if (rumbo < 0) rumbo += 360.0;
    if (rumbo >= 360.0) rumbo -= 360.0;
  }

  // Paquete CSV: contador,x,y,z,rumbo
  String paquete = String(contador) + "," +
                   String(x) + "," +
                   String(y) + "," +
                   String(z) + "," +
                   String(rumbo, 1);

  LoRa.beginPacket();
  LoRa.print(paquete);
  LoRa.endPacket();

  Serial.println(paquete);

  display.clearDisplay();
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.println("TRANSMISOR #" + String(contador));
  display.println("X: " + String(x));
  display.println("Y: " + String(y));
  display.println("Z: " + String(z));
  display.println("Rumbo: " + String(rumbo, 1) + " deg");
  display.display();

  contador++;
  delay(1000);
}
