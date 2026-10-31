#include <SPI.h>
#include <LoRa.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_AHTX0.h>
#include <Adafruit_BMP280.h>

// Pines LoRa del TTGO LoRa32 V1.6
#define LORA_SCK  5
#define LORA_MISO 19
#define LORA_MOSI 27
#define LORA_CS   18
#define LORA_RST  23
#define LORA_DIO0 26
#define LORA_FREQ 433E6

// I2C
#define I2C_SDA 21
#define I2C_SCL 22

Adafruit_SSD1306 display(128, 64, &Wire, -1);
Adafruit_AHTX0 aht;
Adafruit_BMP280 bmp(&Wire);

int contador = 0;

void setup() {
  Serial.begin(115200);
  Wire.begin(I2C_SDA, I2C_SCL);

  // OLED
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println("Error OLED");
    while (true);
  }
  display.setTextColor(WHITE);

  // AHT20
  if (!aht.begin()) {
    Serial.println("Error AHT20");
  }

  // BMP280 (prueba 0x76 y luego 0x77)
  if (!bmp.begin(0x76) && !bmp.begin(0x77)) {
    Serial.println("Error BMP280");
  }

  // LoRa
  SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_CS);
  LoRa.setPins(LORA_CS, LORA_RST, LORA_DIO0);
  if (!LoRa.begin(LORA_FREQ)) {
    Serial.println("Error LoRa");
    while (true);
  }
  Serial.println("Transmisor listo");
}

void loop() {
  sensors_event_t humedad, tempAHT;
  aht.getEvent(&humedad, &tempAHT);

  float tAHT = tempAHT.temperature;
  float hum  = humedad.relative_humidity;
  float tBMP = bmp.readTemperature();
  float pres = bmp.readPressure() / 100.0;     // hPa
  float alt  = bmp.readAltitude(1013.25);      // m

  // Paquete CSV
  String paquete = String(contador) + "," +
                   String(tAHT, 2) + "," +
                   String(hum, 2) + "," +
                   String(tBMP, 2) + "," +
                   String(pres, 2) + "," +
                   String(alt, 2);

  LoRa.beginPacket();
  LoRa.print(paquete);
  LoRa.endPacket();

  Serial.println(paquete);

  // OLED
  display.clearDisplay();
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.println("TRANSMISOR #" + String(contador));
  display.println("T AHT: " + String(tAHT, 1) + " C");
  display.println("Hum:   " + String(hum, 1) + " %");
  display.println("T BMP: " + String(tBMP, 1) + " C");
  display.println("Pres:  " + String(pres, 1) + " hPa");
  display.println("Alt:   " + String(alt, 1) + " m");
  display.display();

  contador++;
  delay(2000);
}
