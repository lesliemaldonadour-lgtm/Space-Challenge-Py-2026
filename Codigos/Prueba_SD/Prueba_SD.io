#include <SPI.h>
#include <SD.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// Pines I2C (OLED)
#define I2C_SDA 21
#define I2C_SCL 22

// Pines microSD (ranura del TTGO LoRa32 V1.6 / módulo SD externo)
#define SD_CS   13
#define SD_MOSI 15
#define SD_MISO 2
#define SD_SCK  14

#define ARCHIVO "/datos.csv"

Adafruit_SSD1306 display(128, 64, &Wire, -1);
SPIClass sdSPI(HSPI);   // bus SPI aparte para no chocar con LoRa

int contador = 0;
bool sdOK = false;

void mostrar(String l1, String l2 = "", String l3 = "") {
  display.clearDisplay();
  display.setCursor(0, 0);
  display.println(l1);
  display.println(l2);
  display.println(l3);
  display.display();
}

void setup() {
  Serial.begin(115200);
  Wire.begin(I2C_SDA, I2C_SCL);

  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println("Error OLED");
    while (true);
  }
  display.setTextColor(WHITE);
  display.setTextSize(1);

  // Iniciar microSD
  sdSPI.begin(SD_SCK, SD_MISO, SD_MOSI, SD_CS);
  if (!SD.begin(SD_CS, sdSPI)) {
    Serial.println("Error: no se pudo iniciar la microSD");
    mostrar("ERROR microSD", "Revisa tarjeta", "y conexiones");
    return;
  }
  sdOK = true;
  Serial.println("microSD lista");

  // Crear el archivo con cabecera si no existe
  if (!SD.exists(ARCHIVO)) {
    File f = SD.open(ARCHIVO, FILE_WRITE);
    if (f) {
      f.println("contador,hola_mundos");
      f.close();
      Serial.println("Archivo creado con cabecera");
    }
  }
  mostrar("microSD OK", "Guardando en", ARCHIVO);
  delay(1500);
}

void loop() {
  if (!sdOK) {
    delay(1000);
    return;
  }

  File f = SD.open(ARCHIVO, FILE_APPEND);
  if (f) {
    f.println(String(contador) + ",HOLA MUNDO");
    f.close();

    Serial.println("Guardado: " + String(contador) + ",HOLA MUNDO");
    mostrar("HOLA MUNDO", "Guardado fila:", String(contador));
    contador++;
  } else {
    Serial.println("Error al abrir el archivo");
    mostrar("ERROR", "No se pudo abrir", ARCHIVO);
  }

  delay(1000);
}
