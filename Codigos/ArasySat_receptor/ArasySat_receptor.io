#include <SPI.h>
#include <LoRa.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// ---------------- Pines LoRa del TTGO LoRa32 V2.1 (T3 V1.6.1) ----------------
#define LORA_SCK   5
#define LORA_MISO  19
#define LORA_MOSI  27
#define LORA_CS    18
#define LORA_RST   23
#define LORA_DIO0  26

// ---------------- OLED ----------------
#define I2C_SDA    21
#define I2C_SCL    22
#define OLED_RST   -1
#define ANCHO      128
#define ALTO       64

#define FRECUENCIA 433E6          // igual que el transmisor
#define CAMPOS_ESPERADOS 26       // N + 20 crudos + 4 filtrados + fase + vibracion (ver transmisor)

// Orden del paquete: N,ax,ay,az,gx,gy,gz,mx,my,mz,tAHT,hum,tBMP,pres,alt,
//                    lat,lon,gpsAlt,sats,vel,rollK,pitchK,altK,velVertK,fase,vibG
// fase: 0 TIERRA, 1 ASCENSO, 2 ESTACIONARIO, 3 DESCENSO, 4 ATERRIZADO

Adafruit_SSD1306 display(ANCHO, ALTO, &Wire, OLED_RST);

const char* CODIGO_FASE[] = {"TIE", "ASC", "EST", "DES", "ATE"};
long ultimoN = -1;
unsigned long recibidos = 0, perdidos = 0;

// Devuelve el campo numero idx (0 = N) de una linea CSV
String campo(const String &s, int idx) {
  int ini = 0;
  for (int i = 0; i < idx; i++) {
    ini = s.indexOf(',', ini);
    if (ini < 0) return "";
    ini++;
  }
  int fin = s.indexOf(',', ini);
  return fin < 0 ? s.substring(ini) : s.substring(ini, fin);
}

void pantallaEspera() {
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(2);
  display.setCursor(0, 0);
  display.println("RECEPTOR");
  display.setTextSize(1);
  display.setCursor(0, 30);
  display.println("Esperando datos...");
  display.display();
}

void mostrarPantalla(long n, int rssi, float snr, const String &p) {
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.print("RX #"); display.print(n);
  display.print("  perd:"); display.println(perdidos);
  display.setCursor(0, 12);
  display.print("RSSI:"); display.print(rssi);
  display.print(" SNR:"); display.println(snr, 1);
  display.setCursor(0, 24);
  display.print("T:"); display.print(campo(p, 10));
  display.print("C H:"); display.print(campo(p, 11)); display.println("%");
  display.setCursor(0, 34);
  display.print("P:"); display.print(campo(p, 13)); display.println("hPa");
  display.setCursor(0, 44);
  display.print("Alt K:"); display.print(campo(p, 22)); display.println("m");
  display.setCursor(0, 54);
  display.print("GPS sats:"); display.print(campo(p, 18));
  int f = campo(p, 24).toInt();
  display.print(" F:"); display.println((f >= 0 && f <= 4) ? CODIGO_FASE[f] : "?");
  display.display();
}

void setup() {
  Serial.begin(115200);   // la pagina web abre el puerto a 115200

  Wire.begin(I2C_SDA, I2C_SCL);
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println("Error al iniciar OLED");
    while (true);
  }
  pantallaEspera();

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

  // Deben ser IGUALES a los del transmisor
  LoRa.setSpreadingFactor(7);
  LoRa.setSignalBandwidth(125E3);
  LoRa.setCodingRate4(5);
  LoRa.setSyncWord(0x12);

  Serial.println("Receptor listo");
}

void loop() {
  int tam = LoRa.parsePacket();
  if (!tam) return;

  String p;
  while (LoRa.available()) p += (char)LoRa.read();
  p.trim();
  int rssi = LoRa.packetRssi();
  float snr = LoRa.packetSnr();

  // Descarta paquetes corruptos o incompletos (el transmisor no usa CRC)
  int comas = 0;
  for (unsigned int i = 0; i < p.length(); i++) if (p[i] == ',') comas++;
  if (comas != CAMPOS_ESPERADOS - 1) {
    Serial.println("Descartado (paquete incompleto): " + p);
    return;
  }

  long n = campo(p, 0).toInt();
  if (ultimoN >= 0 && n > ultimoN + 1) perdidos += n - ultimoN - 1;
  ultimoN = n;
  recibidos++;

  // Formato que espera la pagina web (dos lineas por paquete)
  Serial.println("Recibido: " + p);
  Serial.printf("Paquete #%ld  RSSI: %d dBm  SNR: %.2f\n", n, rssi, snr);

  mostrarPantalla(n, rssi, snr, p);
}
