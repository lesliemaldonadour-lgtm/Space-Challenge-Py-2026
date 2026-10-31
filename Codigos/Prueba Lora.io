#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define OLED_SDA 21
#define OLED_SCL 22
#define OLED_ADDR 0x3C
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

int contador = 0;

void setup() {
  Serial.begin(115200);
  Wire.begin(OLED_SDA, OLED_SCL);

  if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)) {
    Serial.println("No se encontro la pantalla OLED");
    while (true);
  }

  display.clearDisplay();
  display.setTextColor(WHITE);
}

void loop() {
  display.clearDisplay();

  display.setTextSize(2);
  display.setCursor(0, 0);
  display.println("HOLA MUNDO");

  display.setTextSize(1);
  display.setCursor(0, 30);
  display.print("Contador:");

  display.setTextSize(3);
  display.setCursor(0, 42);
  display.print(contador);

  display.display();

  contador++;
  delay(1000);
}
