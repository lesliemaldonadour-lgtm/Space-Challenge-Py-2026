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

// MPU6050: 0x68 (AD0 a GND) o 0x69 (AD0 a 3.3V)
uint8_t addrMPU = 0x68;

Adafruit_SSD1306 display(128, 64, &Wire, -1);

// Sensibilidad (±2 g y ±250 °/s)
const float ACC_SENS  = 16384.0;  // LSB/g
const float GYRO_SENS = 131.0;    // LSB/(°/s)

float roll = 0, pitch = 0;
float offGx = 0, offGy = 0, offGz = 0;
unsigned long tPrev;
unsigned long tEnvio = 0;
int contador = 0;

bool existeI2C(uint8_t addr) {
  Wire.beginTransmission(addr);
  return Wire.endTransmission() == 0;
}

void escribirReg(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(addrMPU);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
}

bool leerMPU(float &ax,
