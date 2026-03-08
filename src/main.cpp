#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_GFX.h>
#define vrx A0 //Joystick pot x
#define vry A1 //Joystick pot y
#define jsw A2 //Joystick switch
#define SDA 20 //I2C data pin
#define SCL 21 //I2C clock pin
#define HOMESWITCH 2 //interrupt for homing axis

//2 sparkfun big easy drivers
#define DR_1 53 //Direction 1
#define EN_1 51 //Enable 1
#define M1_1 49 //MS1 1
#define M2_1 47 //MS2 1
#define M3_1 45 //MS3 1
#define ST_1 43 //Step 1

#define ST_2 52 //Step 2
#define EN_2 50 //Enable 2
#define DR_2 42 //Direction 2
#define M1_2 48 //MS1 2
#define M2_2 46 //MS2 2
#define M3_2 44 //MS3 2

//Definitions for step size
#define fullStep 0
#define halfStep 1
#define quarterStep 2
#define eightStep 3
#define sixteenthStep 4
#define forward 0
#define backward 1

//I2C OLED 128x64
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define SCREEN_ADDRESS 0x3C

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

//Set step size for 
void stepSize(bool driver, uint8_t stepSize) {
  switch (stepSize, driver)
  {
  case 0:
    if(driver) {
      digitalWrite(M1_2, LOW);
      digitalWrite(M2_2, LOW);
      digitalWrite(M3_2, LOW);
    }
    else {
      digitalWrite(M1_1, LOW);
      digitalWrite(M2_1, LOW);
      digitalWrite(M3_1, LOW);
    }
    break;
  
  case 1:
    if(driver) {
      digitalWrite(M1_2, HIGH);
      digitalWrite(M2_2, LOW);
      digitalWrite(M3_2, LOW);
    }
    else {
      digitalWrite(M1_1, HIGH);
      digitalWrite(M2_1, LOW);
      digitalWrite(M3_1, LOW);
    }
    break;
  
  case 2:
    if(driver) {
      digitalWrite(M1_2, LOW);
      digitalWrite(M2_2, HIGH);
      digitalWrite(M3_2, LOW);
    }
    else {
      digitalWrite(M1_1, LOW);
      digitalWrite(M2_1, HIGH);
      digitalWrite(M3_1, LOW);
    }
    break;

  case 3:
    if(driver) {
      digitalWrite(M1_2, HIGH);
      digitalWrite(M2_2, HIGH);
      digitalWrite(M3_2, LOW);
    }
    else {
      digitalWrite(M1_1, HIGH);
      digitalWrite(M2_1, HIGH);
      digitalWrite(M3_1, LOW);
    }
    break;
  
  case 4:
    if(driver) {
      digitalWrite(M1_2, HIGH);
      digitalWrite(M2_2, HIGH);
      digitalWrite(M3_2, HIGH);
    }
    else {
      digitalWrite(M1_1, HIGH);
      digitalWrite(M2_1, HIGH);
      digitalWrite(M3_1, HIGH);
    }
    break;

  default:
    return;
    break;
  }
}

//Step a motor in a direction for i amount of steps.
void callStep(bool driver, bool direction, uint32_t Steps) {
  if (driver)
  {
    digitalWrite(EN_2, LOW);

    if (direction == 1) {
      digitalWrite(DR_2, HIGH);
    }
    else {
      digitalWrite(DR_2,LOW);
    }

    for(uint32_t i = 0; i < Steps; i++) {
      digitalWrite(ST_2, HIGH);
      delay(1);
      digitalWrite(ST_2, LOW);
      delay(1);
    }
  }
  else
  {
    digitalWrite(EN_1, LOW);

    if (direction == 1) {
      digitalWrite(DR_1, HIGH);
    }
    else {
      digitalWrite(DR_1,LOW);
    }

    for(uint32_t i = 0; i < Steps; i++) {
      digitalWrite(ST_1, HIGH);
      delay(1);
      digitalWrite(ST_1, LOW);
      delay(1);
    }
  }  
}

void setup() {
  pinMode(vrx, INPUT);
  pinMode(vry, INPUT);
  pinMode(jsw, INPUT);

  Serial.begin(9600);
  Wire.begin();
  display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS);

  delay(100);
  display.clearDisplay();
  delay(100);
  
}

void OLEDArrowPosition(uint8_t position) {
  switch (position) {
    case 0:
    display.setCursor(100,0);
    break;
    case 1:
    display.setCursor(100,16);
    break;
    case 2:
    display.setCursor(100,32);
    case 3:
    display.setCursor(100, 48);

  }
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


