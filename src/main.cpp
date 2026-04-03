#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_GFX.h>
#include <EncoderButton.h>

/*
Checklist:
  TODO: Check Menus work
  TODO: check motors work and figure out directions
  TODO: Sort Joystick control out and apply expo curve for precise control
  TODO: Figure out linear distance vs raw motor input
  TODO: Figure out Rotation degrees vs raw rotation
  TODO: Display real values on-screen
  TODO: Set up homing feature and make zeroing function
  TODO: Set up record macro feature

*/

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

enum SystemState : uint8_t {
  STATE_MAIN_MENU = 0,
  STATE_MANUAL_MENU,
  STATE_RECORD_MACRO_MENU,
  STATE_JOYSTICK_CTRL
};

struct MenuItem {
  const char* label;
  void (*action)();
};

struct MenuDefinition {
  MenuItem* items;
  uint8_t itemCount;
  SystemState state;
  const MenuDefinition* parent;
};

// Forward declarations
void ArrowPos(uint8_t position);
void drawMenu();
void enterMenu(const MenuDefinition* menu);
MenuItem createBackMenuItem();
void updateJoystickCtrl();
void drawJoystickCtrlScreen();
void actionHomeAxis();
void actionOpenManualMenu();
void actionRecordMacro();
void actionStartMacroRecording();
void actionPlayMacro();
void actionEnterJoystickCtrl();
void actionTestMotors();
void actionGoBack();
void HomeAxis();
void RecordMacroMenu();
void PlayMacro();

MenuItem mainMenu[] = {
  {"Home Axis", actionHomeAxis},
  {"Axes Control", actionOpenManualMenu},
  {"Record Macro", actionRecordMacro},
  {"Test Motors", actionTestMotors}
};

MenuItem manualMenu[] = {
  {"Joystick Ctrl", actionEnterJoystickCtrl},
  createBackMenuItem()
};

MenuItem recordMacroMenu[] = {
  {"Start Recording", actionStartMacroRecording},
  {"Play Macro", actionPlayMacro},
  createBackMenuItem()
};

const MenuDefinition mainMenuDef = {
  mainMenu,
  sizeof(mainMenu) / sizeof(mainMenu[0]),
  STATE_MAIN_MENU,
  nullptr
};

const MenuDefinition manualMenuDef = {
  manualMenu,
  sizeof(manualMenu) / sizeof(manualMenu[0]),
  STATE_MANUAL_MENU,
  &mainMenuDef
};

const MenuDefinition recordMacroMenuDef = {
  recordMacroMenu,
  sizeof(recordMacroMenu) / sizeof(recordMacroMenu[0]),
  STATE_RECORD_MACRO_MENU,
  &mainMenuDef
};


uint8_t cursorPosition = 0;
const MenuDefinition* activeMenuDef = &mainMenuDef;
const MenuDefinition* joystickReturnMenuDef = &manualMenuDef;
SystemState currentState = STATE_MAIN_MENU;
bool menuPressed = false;       // set true when encoder button is clicked
bool isHomed = false;
bool menuNeedsRedraw = true;
uint32_t linearDistanceRaw = 0;
float linearDistanceMM = 0;
uint16_t rotationDistance = 0;
bool joystickSwitchState = HIGH;
uint32_t joystickSwitchLastEdgeMs = 0;
const uint16_t JOY_SWITCH_DEBOUNCE_MS = 40;

void onEb1Clicked(EncoderButton& eb) {
  menuPressed = true;  // caller checks this flag to act on the selected item
  Serial.println("Encoder Button Clicked!");
}

void onEb1Encoder(EncoderButton& eb) {
  int8_t delta = eb.increment();
  int8_t newPos = (int8_t)cursorPosition + delta;
  uint8_t limit = activeMenuDef->itemCount;
  
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
  if (position >= activeMenuDef->itemCount) {
    return;
  }

  display.setCursor(100, (position * 16) + 8);
  display.print(F("<--"));
  cursorPosition = position;
  display.display();
}

void drawMenu() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(1);
  for (uint8_t i = 0; i < activeMenuDef->itemCount; i++) {
    display.setCursor(8, (i * 16) + 8);
    display.print(activeMenuDef->items[i].label);
  }
  display.display();
}

void enterMenu(const MenuDefinition* menu) {
  activeMenuDef = menu;
  currentState = menu->state;
  cursorPosition = 0;
  menuNeedsRedraw = true;
}

MenuItem createBackMenuItem() {
  MenuItem backItem = {"Back", actionGoBack};
  return backItem;
}

void HomeAxis(){
  //move linear in opposite direction for a few steps, then move towards home until it hits the interrupt homing microswitch.
  stepSize(fullStep, 1); //not sure on driver number yet
  callStep(1,0,100); //not sure on driver or direction yet
}

void drawJoystickCtrlScreen() {
  display.clearDisplay();
  display.setTextSize(1);
  display.drawRect(6, 4, 116, 14, SSD1306_WHITE);
  display.fillRect(7, 5, 114, 12, SSD1306_WHITE);
  display.setTextColor(SSD1306_BLACK);
  display.setCursor(34, 8);
  display.print(F("Joystick Ctrl"));

  display.setTextColor(SSD1306_WHITE);
  display.setCursor(8, 56);
  display.print(F("Press Joy to exit"));
  display.display();
}

void updateJoystickCtrl() {
  int xVal = analogRead(vrx);
  int yVal = analogRead(vry);

  const int16_t joyCenter = 512;
  const int16_t deadzone = 45;
  const uint8_t maxStepsPerUpdate = 12;

  auto axisToSteps = [&](int16_t axisOffset) -> uint32_t {
    int16_t absOffset = abs(axisOffset);
    if (absOffset <= deadzone) {
      return 0;
    }

    float normalized = (float)(absOffset - deadzone) / (float)(511 - deadzone);
    if (normalized > 1.0f) {
      normalized = 1.0f;
    }

    // Near-linear feel with gentle acceleration at the very end of travel.
    float curved = (0.85f * normalized) + (0.15f * normalized * normalized * normalized);
    return 1 + (uint32_t)(curved * (maxStepsPerUpdate - 1));
  };

  auto axisToStepMode = [&](int16_t axisOffset) -> uint8_t {
    int16_t absOffset = abs(axisOffset);
    if (absOffset <= deadzone) {
      return sixteenthStep;
    }

    float normalized = (float)(absOffset - deadzone) / (float)(511 - deadzone);
    if (normalized > 1.0f) {
      normalized = 1.0f;
    }

    // Keep fine control over almost all travel; reserve full-step for extremes.
    if (normalized < 0.68f) return sixteenthStep;
    if (normalized < 0.86f) return eightStep;
    if (normalized < 0.94f) return quarterStep;
    if (normalized < 0.98f) return halfStep;
    return fullStep;
  };

  int16_t xOffset = xVal - joyCenter;
  int16_t yOffset = yVal - joyCenter;

  uint32_t linearSteps = axisToSteps(xOffset);
  uint32_t rotationSteps = axisToSteps(yOffset);
  uint8_t linearMode = axisToStepMode(xOffset);
  uint8_t rotationMode = axisToStepMode(yOffset);

  // Joy X: toward 0 => linear right, toward 1024 => linear left.
  if (linearSteps > 0) {
    stepSize(linearMode, 1);
    if (xOffset < 0) {
      callStep(1, 0, linearSteps);
    } else {
      callStep(1, 1, linearSteps);
    }
  }

  // Joy Y: toward 0 => anticlockwise, toward 1024 => clockwise.
  if (rotationSteps > 0) {
    stepSize(rotationMode, 0);
    if (yOffset < 0) {
      callStep(0, 0, rotationSteps);
    } else {
      callStep(0, 1, rotationSteps);
    }
  }

  bool jsStateNow = digitalRead(jsw);
  uint32_t nowMs = millis();
  bool exitPressed = false;

  if (jsStateNow != joystickSwitchState && (nowMs - joystickSwitchLastEdgeMs) >= JOY_SWITCH_DEBOUNCE_MS) {
    joystickSwitchState = jsStateNow;
    joystickSwitchLastEdgeMs = nowMs;

    // With INPUT_PULLUP, LOW is a valid press event.
    if (joystickSwitchState == LOW) {
      exitPressed = true;
    }
  }

  if (exitPressed) {
    enterMenu(joystickReturnMenuDef);
  }
}

void actionHomeAxis() { HomeAxis(); }

void actionOpenManualMenu() {
  enterMenu(&manualMenuDef);
}

void actionRecordMacro() {
  enterMenu(&recordMacroMenuDef);
}

void actionStartMacroRecording() {
  RecordMacroMenu();
}

void actionPlayMacro() {
  PlayMacro();
}

void actionEnterJoystickCtrl() {
  joystickReturnMenuDef = activeMenuDef;
  joystickSwitchState = digitalRead(jsw);
  joystickSwitchLastEdgeMs = millis();
  stepSize(fullStep, 0);
  stepSize(fullStep, 1);
  currentState = STATE_JOYSTICK_CTRL;
  drawJoystickCtrlScreen();
}

void testMotors() {
  stepSize(fullStep,0);
  callStep(0,0,250); //Rotational motor going anticlockwise
  callStep(0,1,250); //Rotational motor going clockwise
  delay(1000);
  stepSize(fullStep,1);
  callStep(1,0,250); //Linear Axis going right
  callStep(1,1,250); //Linear Axis going Left
}


void actionTestMotors() {
  testMotors();
}

void actionGoBack() {
  if (activeMenuDef->parent != nullptr) {
    enterMenu(activeMenuDef->parent);
  }
}




void RecordMacroMenu() {
  Serial.println("Start macro recording requested");
}

void PlayMacro() {
  Serial.println("Play macro requested");
}

void setup() {
  pinMode(vrx, INPUT);
  pinMode(vry, INPUT);
  pinMode(jsw, INPUT_PULLUP);

  for(int i = 42; i<54; i++) {
    pinMode(i, OUTPUT);
    digitalWrite(i, LOW);
  }
  digitalWrite(EN_1, HIGH);
  digitalWrite(EN_2, HIGH);

  Serial.begin(9600);
  Wire.begin();
  display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS);

  delay(100);
  display.clearDisplay();
  delay(100);
  eb1.setClickHandler(onEb1Clicked);
  eb1.setEncoderHandler(onEb1Encoder);
  joystickSwitchState = digitalRead(jsw);
  joystickSwitchLastEdgeMs = millis();

  Serial.println("System Ready");
  testMotors();
  Serial.println("Testing Motors");

  
}



void loop() {
  eb1.update();

  if (currentState == STATE_JOYSTICK_CTRL) {
    updateJoystickCtrl();
    return;
  }

  if (menuNeedsRedraw) {
    drawMenu();
    ArrowPos(cursorPosition);
    menuNeedsRedraw = false;
  }

  if (menuPressed) {
    menuPressed = false;
    if (cursorPosition < activeMenuDef->itemCount && activeMenuDef->items[cursorPosition].action != nullptr) {
      activeMenuDef->items[cursorPosition].action();
    }
  }

}


