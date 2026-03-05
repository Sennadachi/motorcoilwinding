#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_GFX.h>
#define vrx A0
#define vry A1
#define jsw A2
#define SDA 20
#define SCL 21
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define SCREEN_ADDRESS 0x3C
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);



void setup() {
  pinMode(vrx, INPUT);
  pinMode(vry, INPUT);
  pinMode(jsw, INPUT);

  Serial.begin(9600);
  Wire.begin();
  display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS);

  delay(100);
  display.clearDisplay();
  delay(100);aa
  
}

void loop() {

  display.clearDisplay();

  display.setTextSize(1);
  display.setCursor(1, (display.height()/2));
  display.setTextColor(1);
  display.println(F("Hello world"));
  display.display();
  delay(3000);
}


