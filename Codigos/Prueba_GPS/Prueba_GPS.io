#include <SPI.h>
#include <LoRa.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <TinyGPSPlus.h>

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

// GPS NEO-6M
#define GPS_RX_PIN 34   // pin del ESP32 que recibe (va al TX del GPS)
#define GPS_TX_PIN 4    // pin del ESP32 que transmite (va al RX del GPS)
#define GPS_BAUD   9600

Adafruit_SSD1306 display(128, 64, &Wire, -1);
TinyGPSPlus gps;
HardwareSerial gpsSerial(1);

unsigned long tEnvio = 0;
int contador = 0;

void setup() {
  Serial.begin(115200);
  Wire.begin(I2C_SDA, I2C_SCL);

  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println("Error OLED");
    while (true);
  }
  display.setTextColor(WHITE);
  display.setTextSize(1);

  gpsSerial.begin(GPS_BAUD, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);

  SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_CS);
  LoRa.setPins(LORA_CS, LORA_RST, LORA_DIO0);
  if (!LoRa.begin(LORA_FREQ)) {
    Serial.println("Error LoRa");
    while (true);
  }
  Serial.println("Transmisor listo");
}

void loop() {
  // Leer todo lo que llega del GPS
  while (gpsSerial.available()) {
    gps.encode(gpsSerial.read());
  }

  // Enviar y mostrar cada 2 segundos
  if (millis() - tEnvio >= 2000) {
    tEnvio = millis();

    bool fix = gps.location.isValid();
    double lat = fix ? gps.location.lat() : 0;
    double lon = fix ? gps.location.lng() : 0;
    double alt = gps.altitude.isValid() ? gps.altitude.meters() : 0;
    double vel = gps.speed.isValid() ? gps.speed.kmph() : 0;
    int sats   = gps.satellites.isValid() ? gps.satellites.value() : 0;
    double hdop = gps.hdop.isValid() ? gps.hdop.hdop() : 0;

    char hora[12] = "--:--:--";
    if (gps.time.isValid()) {
      sprintf(hora, "%02d:%02d:%02d", gps.time.hour(), gps.time.minute(), gps.time.second());
    }

    // CSV: contador,fix,lat,lon,alt,vel,sats,hdop,hora(UTC)
    String paquete = String(contador) + "," +
                     String(fix ? 1 : 0) + "," +
                     String(lat, 6) + "," +
                     String(lon, 6) + "," +
                     String(alt, 1) + "," +
                     String(vel, 1) + "," +
                     String(sats) + "," +
                     String(hdop, 1) + "," +
                     String(hora);

    LoRa.beginPacket();
    LoRa.print(paquete);
    LoRa.endPacket();

    Serial.println(paquete);
    Serial.println("Caracteres GPS procesados: " + String(gps.charsProcessed()));

    display.clearDisplay();
    display.setCursor(0, 0);
    display.println("TX #" + String(contador) + (fix ? "  FIX" : "  SIN FIX"));
    display.println("Lat: " + String(lat, 6));
    display.println("Lon: " + String(lon, 6));
    display.println("Alt: " + String(alt, 1) + " m");
    display.println("Vel: " + String(vel, 1) + " km/h");
    display.println("Sats: " + String(sats) + " HDOP: " + String(hdop, 1));
    display.println("UTC: " + String(hora));
    display.display();

    contador++;
  }
}
