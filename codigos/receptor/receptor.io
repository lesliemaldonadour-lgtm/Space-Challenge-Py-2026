#include <SPI.h>
#include <LoRa.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// Pines LoRa del TTGO LoRa32 V2.1 (T3 V1.6.1)
#define LORA_SCK   5
#define LORA_MISO  19
#define LORA_MOSI  27
#define LORA_CS    18
#define LORA_RST   23
#define LORA_DIO0  26

// Pines OLED (SSD1306 128x64, I2C)
#define OLED_SDA   21
#define OLED_SCL   22
#define OLED_RST   -1
#define ANCHO      128
#define ALTO       64

#define FRECUENCIA 433E6   // 433 MHz
#define NUM_CAMPOS 24
#define CAMBIO_PAGINA 3000 // ms entre paginas de la pantalla

Adafruit_SSD1306 display(ANCHO, ALTO, &Wire, OLED_RST);

// Campos: 0=N 1-3=acel(g) 4-6=giro(dps) 7-9=mag(uT) 10=tAHT 11=hum
//         12=tBMP 13=presion 14=alt 15=lat 16=lon 17=altGPS 18=sats 19=vel
// Filtrados (Kalman): 20=roll 21=pitch 22=altitud 23=vel. vertical
String campos[NUM_CAMPOS];
bool hayDatos = false;
int rssiActual = 0;
int pagina = 0;
unsigned long ultimaPagina = 0;

int separar(const String &s, String *out, int maximo) {
  int n = 0, ini = 0;
  while (n < maximo) {
    int pos = s.indexOf(',', ini);
    if (pos < 0) { out[n++] = s.substring(ini); break; }
    out[n++] = s.substring(ini, pos);
    ini = pos + 1;
  }
  return n;
}

void dibujar() {
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(1);

  if (!hayDatos) {
    display.setTextSize(2);
    display.setCursor(0, 0);
    display.println("RECEPTOR");
    display.setTextSize(1);
    display.setCursor(0, 24);
    display.println("433 MHz");
    display.setCursor(0, 38);
    display.println("Esperando datos...");
    display.display();
    return;
  }

  // Cabecera
  display.setCursor(0, 0);
  display.print("RX #"); display.print(campos[0]);
  display.print(" "); display.print(rssiActual); display.print("dBm");
  display.drawLine(0, 9, 127, 9, SSD1306_WHITE);

  display.setCursor(0, 13);
  if (pagina == 0) {
    display.println("AMBIENTE");
    display.print("Temp: "); display.print(campos[10]); display.println(" C");
    display.print("Hum:  "); display.print(campos[11]); display.println(" %");
    display.print("Pres: "); display.print(campos[13]); display.println(" hPa");
    display.print("Alt:  "); display.print(campos[14]); display.println(" m");
  } else if (pagina == 1) {
    display.println("MOVIMIENTO (g / dps)");
    display.print("A:"); display.print(campos[1]); display.print(" ");
    display.print(campos[2]); display.print(" "); display.println(campos[3]);
    display.print("G:"); display.print(campos[4]); display.print(" ");
    display.print(campos[5]); display.print(" "); display.println(campos[6]);
    display.println("MAG (uT)");
    display.print(campos[7]); display.print(" "); display.print(campos[8]);
    display.print(" "); display.println(campos[9]);
  } else if (pagina == 2) {
    display.println("GPS");
    display.print("Lat: "); display.println(campos[15]);
    display.print("Lon: "); display.println(campos[16]);
    display.print("Alt:"); display.print(campos[17]);
    display.print(" Sat:"); display.println(campos[18]);
    display.print("Vel: "); display.print(campos[19]); display.println(" km/h");
  } else {
    display.println("FILTRADO (Kalman)");
    display.print("Roll:  "); display.print(campos[20]); display.println(" deg");
    display.print("Pitch: "); display.print(campos[21]); display.println(" deg");
    display.print("Alt:   "); display.print(campos[22]); display.println(" m");
    display.print("Vel V: "); display.print(campos[23]); display.println(" m/s");
  }
  display.display();
}

void setup() {
  Serial.begin(115200);

  // OLED
  Wire.begin(OLED_SDA, OLED_SCL);
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println("Error al iniciar OLED");
    while (true);
  }
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(2);
  display.setCursor(0, 0);
  display.println("RECEPTOR");
  display.setTextSize(1);
  display.setCursor(0, 30);
  display.println("Iniciando LoRa...");
  display.display();

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

  // Deben coincidir con el transmisor
  LoRa.setSpreadingFactor(7);
  LoRa.setSignalBandwidth(125E3);
  LoRa.setCodingRate4(5);
  LoRa.setSyncWord(0x12);

  Serial.println("Receptor listo, esperando datos...");
  dibujar();
}

void loop() {
  int tamano = LoRa.parsePacket();
  if (tamano) {
    String recibido = "";
    while (LoRa.available()) {
      recibido += (char)LoRa.read();
    }
    rssiActual = LoRa.packetRssi();

    for (int i = 0; i < NUM_CAMPOS; i++) campos[i] = "-";
    int n = separar(recibido, campos, NUM_CAMPOS);

    Serial.println("Recibido: " + recibido);
    if (n == NUM_CAMPOS) {
      hayDatos = true;
      Serial.println("----------------------------------------");
      Serial.println("Paquete #" + campos[0] + "  RSSI: " + String(rssiActual) +
                     " dBm  SNR: " + String(LoRa.packetSnr()));
      Serial.println("Acel (g):   " + campos[1] + ", " + campos[2] + ", " + campos[3]);
      Serial.println("Giro (dps): " + campos[4] + ", " + campos[5] + ", " + campos[6]);
      Serial.println("Mag (uT):   " + campos[7] + ", " + campos[8] + ", " + campos[9]);
      Serial.println("AHT20:      " + campos[10] + " C, " + campos[11] + " %HR");
      Serial.println("BMP280:     " + campos[12] + " C, " + campos[13] + " hPa, " + campos[14] + " m");
      Serial.println("GPS:        " + campos[15] + ", " + campos[16] + "  alt " + campos[17] +
                     " m  sats " + campos[18] + "  vel " + campos[19] + " km/h");
      Serial.println("Kalman:     roll " + campos[20] + ", pitch " + campos[21] +
                     " deg  alt " + campos[22] + " m  vel vert " + campos[23] + " m/s");
    } else {
      Serial.println("Paquete con formato inesperado (" + String(n) + " campos)");
    }
    dibujar();
  }

  if (hayDatos && millis() - ultimaPagina > CAMBIO_PAGINA) {
    ultimaPagina = millis();
    pagina = (pagina + 1) % 4;
    dibujar();
  }
}
