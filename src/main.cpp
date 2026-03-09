#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_GFX.h>
#include <EncoderButton.h>

#define vrx A0 //Joystick pot x
#define vry A1 //Joystick pot y
#define jsw A2 //Joystick switch
#define SDA 20 //I2C data pin
#define SCL 21 //I2C clock pin
#define HOMESWITCH 2 //interrupt for homing axis
#define ENCA 18
#define ENCB 19
#define ENCSW 3

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
EncoderButton eb1(ENCA, ENCB, ENCSW);

// Forward declaration
void ArrowPos(uint8_t position);


uint8_t cursorPosition = 0;
const uint8_t MENU_ITEMS = 3;  // number of items in the current menu
bool menuPressed = false;       // set true when encoder button is clicked
bool isHomed = false;
bool backFlag = false;
uint8_t currentMenu = 0;
uint32_t linearDistanceRaw = 0;
float linearDistanceMM = 0;
uint16_t rotationDistance = 0;

void onEb1Clicked(EncoderButton& eb) {
  menuPressed = true;  // caller checks this flag to act on the selected item
  Serial.println("Encoder Button Clicked!");
}

void onEb1Encoder(EncoderButton& eb) {
  int8_t delta = eb.increment();
  int8_t newPos = (int8_t)cursorPosition + delta;
  
  // Decide limit based on current menu
  uint8_t limit = (currentMenu == 0) ? 3 : 3; // Both currently have 3 items
  
  if (newPos < 0) newPos = 0;
  if (newPos >= limit) newPos = limit - 1;
  
  if (newPos != (int8_t)cursorPosition) {
    ArrowPos((uint8_t)newPos);
  }
}

//Set step size for 
void stepSize(uint8_t stepSize, bool driver) {
  switch (stepSize)
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

    if (direction) {
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

    if (direction) {
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

void ArrowPos(uint8_t position) {
  display.setTextSize(1);
  display.setTextColor(1);
  display.fillRect(100,0,28,64,SSD1306_BLACK);
  display.display();
  switch (position) {
    case 0:
      display.setCursor(100, 0+8); //8 offset
      display.print(F("<--"));
      cursorPosition = 0;
    break;
    case 1:
      display.setCursor(100, 16+8);
      display.print(F("<--"));
      cursorPosition= 1;
    break;
    case 2:
      display.setCursor(100, 32+8);
      display.print(F("<--"));
      cursorPosition = 2;
    break;
    case 3:
      display.setCursor(100, 48+8);
      display.print(F("<--"));
      cursorPosition = 3;
    break;
    default:
      return;
    break;
  }
  display.display();
}

void drawHomeMenu(){
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(1);
  display.setCursor(8, 8);
  display.print(F("Home Axis"));
  display.setCursor(8, 16+8);
  display.print(F("Axes Control"));
  display.setCursor(8, 32+8);
  display.print(F("Record Macro"));
  display.display();
}

void HomeAxis(){
  //move linear in opposite direction for a few steps, then move towards home until it hits the interrupt homing microswitch.
  stepSize(fullStep, 1); //not sure on driver number yet
  callStep(1,0,100); //not sure on driver or direction yet
}

void drawManualControlMenu(){
  currentMenu = 1;      
  cursorPosition = 0;
  backFlag = false;
  display.setTextColor(1); //white
  display.clearDisplay();
  display.setCursor(8, 8);
  display.print(F("Joystick Control"));
  display.setCursor(8, 16+8);
  display.print(F("Back"));
  display.display();
  ArrowPos(0); // Draw initial arrow for this menu
}

void joyCTRL(){
  display.setCursor(8,8);
  display.fillRect(8,8,100,7,1);
  display.setTextColor(0);
  display.print(F("Joystick Ctrl"));
  bool exitCTRL = false;
  while (!exitCTRL) {
    int xVal = analogRead(vrx);
    int yVal = analogRead(vry);

    Serial.println("Joy X: ");
    Serial.println(xVal);
    Serial.println("Joy Y:");
    Serial.println(yVal);

    //TODO: motor logic - use vrx for linear axis, and vry for rotation,  with expo curved inputs
    //TODO: update linear and rotation distance variables and calculate mm and degrees using algorithm in a seperate function (real testing rq)
    //TODO: then display distances on UI

    if (digitalRead(jsw) == LOW)
    {
      delay(200);
      drawManualControlMenu();
      exitCTRL = true;
    }
  }
  return;
}

void manualControlMenu(){
  drawManualControlMenu();
  
  while (!backFlag) {
    eb1.update(); // Keep library heartbeat alive
    
    if (menuPressed) {  
      menuPressed = false;
      if (cursorPosition == 0) {
        joyCTRL();
      }
      else if(cursorPosition == 1) {
        backFlag = true;
      }
    }
  }
  // Returning to main menu
  currentMenu = 0;
  drawHomeMenu();
  ArrowPos(0);
}







void RecordMacroMenu(){}

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
  eb1.setClickHandler(onEb1Clicked);
  eb1.setEncoderHandler(onEb1Encoder);

  drawHomeMenu();
  ArrowPos(0);
  
}



void loop() {
  eb1.update();

  if (menuPressed) {
    menuPressed = false;
    switch (cursorPosition) {
      case 0: HomeAxis(); Serial.println("Home Axis Pressed");
        break;
      case 1: manualControlMenu(); Serial.println("Axes Control Pressed");
        break;
      case 2: RecordMacroMenu(); Serial.println("Record Macro pressed");
        break;
    }
  }

}


