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
  TODO: Figure out linear distance vs raw motor input 1880 steps = 37.5mm
  TODO: Figure out Rotation degrees vs raw rotation 600 = 360 degrees
  TODO: Display real values on-screen
  TODO: Set up homing feature and make zeroing function
  TODO: Set up rotation counter
  TODO: Set up record macro feature <--

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
void drawJoystickDebugReadout();
float rawLinearToMM(int32_t rawSixteenthUnits);
int16_t rawRotationToDegrees(int32_t rawSixteenthUnits);
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
void beginMacroRecording();
void recordMacroMove(bool driver, bool direction, uint32_t steps, uint8_t stepMode);

MenuItem mainMenu[] = {
  {"Home Axis", actionHomeAxis},
  {"Axes Control", actionOpenManualMenu},
  {"Macros", actionRecordMacro},
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
int32_t linearDistanceRaw = 0;
float linearDistanceMM = 0;
int32_t rotationDistanceRaw = 0;
int16_t rotationDistance = 0;
uint32_t rotationTurnCount = 0;
uint32_t rotationAbsRemainderRaw = 0;
int32_t joystickLinearFullStepEqFromStart = 0;
int32_t joystickRotationFullStepEqFromStart = 0;
bool joystickSwitchState = HIGH;
uint32_t joystickSwitchLastEdgeMs = 0;
const uint16_t JOY_SWITCH_DEBOUNCE_MS = 40;
const float LINEAR_MM_PER_FULL_STEP = 37.5f / 1880.0f;
const float ROTATION_DEG_PER_FULL_STEP = 360.0f / 600.0f;
const uint32_t ROTATION_RAW_UNITS_PER_REV = 600UL * 16UL;
const uint32_t HOME_SWITCH_ISR_DEBOUNCE_US = 3000;
const uint16_t MACRO_PLAYBACK_PULSE_DELAY_MS = 3;
const uint16_t MACRO_PLAYBACK_BETWEEN_MOVES_MS = 8;
const uint16_t MACRO_NEAR_LIMIT_MARGIN = 20;

volatile bool linearStopRequested = false;
volatile bool homeSwitchTriggered = false;
volatile bool homeZ1eroRequested = false;
volatile bool homingProcedureActive = false;
volatile uint32_t homeSwitchLastIsrUs = 0;

const uint16_t MAX_MACRO_MOVES = 600;

struct MacroMove {
  bool driver;
  bool direction;
  uint32_t steps;
  uint8_t stepMode;
};

MacroMove recordedMacro[MAX_MACRO_MOVES];
uint16_t recordedMacroCount = 0;
bool macroRecordingActive = false;
bool macroRecordingOverflow = false;
bool macroNearLimitWarned = false;
bool macroLimitHitLatched = false;
bool macroStopByLimitRequested = false;

void onHomeSwitchChange();
void processHomeSwitchEvents();

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

void onHomeSwitchChange() {
  uint32_t nowUs = micros();
  if ((uint32_t)(nowUs - homeSwitchLastIsrUs) < HOME_SWITCH_ISR_DEBOUNCE_US) {
    return;
  }

  if (digitalRead(HOMESWITCH) == LOW) {
    homeSwitchLastIsrUs = nowUs;
    homeSwitchTriggered = true;
    linearStopRequested = true;
    if (homingProcedureActive) {
      homeZeroRequested = true;
    }
  }
}

void processHomeSwitchEvents() {
  bool doZero = false;

  noInterrupts();
  if (homeZeroRequested) {
    homeZeroRequested = false;
    doZero = true;
  }
  interrupts();

  if (doZero) {
    joystickLinearFullStepEqFromStart = 0;
    linearDistanceRaw = 0;
    linearDistanceMM = 0.0f;
    isHomed = true;
    Serial.println("Home switch hit during homing. Linear position set to zero.");
  }
}

//Set step size for motor drivers
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

void beginMacroRecording() {
  recordedMacroCount = 0;
  macroRecordingOverflow = false;
  macroRecordingActive = true;
  macroNearLimitWarned = false;
  macroLimitHitLatched = false;
  macroStopByLimitRequested = false;
  Serial.println("Macro recording started. Use joystick, then press joystick button to stop.");

  joystickReturnMenuDef = &recordMacroMenuDef;
  joystickSwitchState = digitalRead(jsw);
  joystickSwitchLastEdgeMs = millis();
  joystickLinearFullStepEqFromStart = 0;
  joystickRotationFullStepEqFromStart = 0;
  stepSize(fullStep, 0);
  stepSize(fullStep, 1);
  currentState = STATE_JOYSTICK_CTRL;
  drawJoystickCtrlScreen();
  drawJoystickDebugReadout();
}

void recordMacroMove(bool driver, bool direction, uint32_t steps, uint8_t stepMode) {
  if (!macroRecordingActive || steps == 0) {
    return;
  }

  if (!macroNearLimitWarned) {
    uint16_t warnStart = (MAX_MACRO_MOVES > MACRO_NEAR_LIMIT_MARGIN)
      ? (MAX_MACRO_MOVES - MACRO_NEAR_LIMIT_MARGIN)
      : 0;
    if (recordedMacroCount >= warnStart) {
      macroNearLimitWarned = true;
      Serial.print("Warning: macro buffer almost full. Remaining slots: ");
      Serial.println(MAX_MACRO_MOVES - recordedMacroCount);
    }
  }

  // Merge consecutive identical moves to reduce macro entry usage.
  if (recordedMacroCount > 0) {
    MacroMove& lastMove = recordedMacro[recordedMacroCount - 1];
    if (lastMove.driver == driver &&
        lastMove.direction == direction &&
        lastMove.stepMode == stepMode) {
      if (lastMove.steps <= (UINT32_MAX - steps)) {
        lastMove.steps += steps;
        return;
      }
      macroRecordingOverflow = true;
      return;
    }
  }

  if (recordedMacroCount >= MAX_MACRO_MOVES) {
    macroRecordingOverflow = true;
    macroRecordingActive = false;
    macroStopByLimitRequested = true;
    if (!macroLimitHitLatched) {
      macroLimitHitLatched = true;
      Serial.print("Macro recording limit reached. Macro saved with moves: ");
      Serial.println(recordedMacroCount);
    }
    return;
  }

  recordedMacro[recordedMacroCount].driver = driver;
  recordedMacro[recordedMacroCount].direction = direction;
  recordedMacro[recordedMacroCount].steps = steps;
  recordedMacro[recordedMacroCount].stepMode = stepMode;
  recordedMacroCount++;

  if (recordedMacroCount >= MAX_MACRO_MOVES) {
    macroRecordingOverflow = true;
    macroRecordingActive = false;
    macroStopByLimitRequested = true;
    if (!macroLimitHitLatched) {
      macroLimitHitLatched = true;
      Serial.print("Macro recording limit reached. Macro saved with moves: ");
      Serial.println(recordedMacroCount);
    }
  }
}



//Step a motor in a direction for i amount of steps.
void callStep(bool driver, bool direction, uint32_t Steps, uint16_t pulseDelayMs = 1) {
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
      if (linearStopRequested) {
        break;
      }
      digitalWrite(ST_2, HIGH);
      delay(pulseDelayMs);
      digitalWrite(ST_2, LOW);
      delay(pulseDelayMs);
    }

    if (linearStopRequested) {
      digitalWrite(EN_2, HIGH);
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
      delay(pulseDelayMs);
      digitalWrite(ST_1, LOW);
      delay(pulseDelayMs);
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
  // Move off the switch slightly, then move towards home until ISR requests stop.
  noInterrupts();
  homingProcedureActive = true;
  linearStopRequested = false;
  homeSwitchTriggered = false;
  homeZeroRequested = false;
  interrupts();

  // Reset rotation turn count when homing is requested.
  rotationTurnCount = 0;
  rotationAbsRemainderRaw = 0;

  stepSize(fullStep, 1);

  // Back off first.
  callStep(1, 0, 100);

  // Seek home switch and stop immediately when ISR latches LOW.
  for (uint32_t i = 0; i < 60000; i++) {
    if (linearStopRequested) {
      break;
    }
    callStep(1, 1, 1);
  }

  noInterrupts();
  homingProcedureActive = false;
  linearStopRequested = false;
  interrupts();

  processHomeSwitchEvents();
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
  if (macroRecordingActive) {
    display.setCursor(8, 20);
    display.print(F("REC"));
  }
  display.setCursor(8, 56);
  display.print(F("Press Joy to exit"));
  display.display();
}

void drawJoystickDebugReadout() {
  display.fillRect(8, 22, 112, 34, SSD1306_BLACK);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(8, 24);
  display.print(F("Lin mm: "));
  display.print(linearDistanceMM, 2);
  display.setCursor(8, 36);
  display.print(F("Rot deg: "));
  display.print(rotationDistance);
  display.setCursor(8, 48);
  display.print(F("Rot cnt: "));
  display.print(rotationTurnCount);
  display.display();
}

float rawLinearToMM(int32_t rawSixteenthUnits) {
  return ((float)rawSixteenthUnits / 16.0f) * LINEAR_MM_PER_FULL_STEP;
}

int16_t rawRotationToDegrees(int32_t rawSixteenthUnits) {
  float degrees = ((float)rawSixteenthUnits / 16.0f) * ROTATION_DEG_PER_FULL_STEP;
  return (int16_t)lroundf(degrees);
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

  auto modeToSixteenthPerPulse = [&](uint8_t mode) -> int32_t {
    switch (mode) {
      case fullStep: return 16;
      case halfStep: return 8;
      case quarterStep: return 4;
      case eightStep: return 2;
      case sixteenthStep: return 1;
      default: return 16;
    }
  };

  int32_t linearUnitsPerPulse = modeToSixteenthPerPulse(linearMode);
  int32_t rotationUnitsPerPulse = modeToSixteenthPerPulse(rotationMode);

  // Joy X: toward 0 => linear right, toward 1024 => linear left.
  if (linearSteps > 0) {
    stepSize(linearMode, 1);
    if (xOffset < 0) {
      recordMacroMove(1, 0, linearSteps, linearMode);
      callStep(1, 0, linearSteps);
      joystickLinearFullStepEqFromStart += ((int32_t)linearSteps * linearUnitsPerPulse);
    } else {
      recordMacroMove(1, 1, linearSteps, linearMode);
      callStep(1, 1, linearSteps);
      joystickLinearFullStepEqFromStart -= ((int32_t)linearSteps * linearUnitsPerPulse);
    }
  }

  // Joy Y: toward 0 => anticlockwise, toward 1024 => clockwise.
  if (rotationSteps > 0) {
    stepSize(rotationMode, 0);
    uint32_t rotationDeltaAbsRaw = (uint32_t)rotationSteps * (uint32_t)rotationUnitsPerPulse;
    rotationAbsRemainderRaw += rotationDeltaAbsRaw;
    while (rotationAbsRemainderRaw >= ROTATION_RAW_UNITS_PER_REV) {
      rotationAbsRemainderRaw -= ROTATION_RAW_UNITS_PER_REV;
      rotationTurnCount++;
    }

    if (yOffset < 0) {
      recordMacroMove(0, 0, rotationSteps, rotationMode);
      callStep(0, 0, rotationSteps);
      joystickRotationFullStepEqFromStart += ((int32_t)rotationSteps * rotationUnitsPerPulse);
    } else {
      recordMacroMove(0, 1, rotationSteps, rotationMode);
      callStep(0, 1, rotationSteps);
      joystickRotationFullStepEqFromStart -= ((int32_t)rotationSteps * rotationUnitsPerPulse);
    }
  }

  linearDistanceRaw = joystickLinearFullStepEqFromStart;
  rotationDistanceRaw = joystickRotationFullStepEqFromStart;
  linearDistanceMM = rawLinearToMM(linearDistanceRaw);
  rotationDistance = rawRotationToDegrees(rotationDistanceRaw);

  drawJoystickDebugReadout();

  if (macroStopByLimitRequested) {
    macroStopByLimitRequested = false;
    enterMenu(joystickReturnMenuDef);
    return;
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
    if (macroRecordingActive) {
      macroRecordingActive = false;
      if (macroRecordingOverflow) {
        Serial.print("Macro recording stopped (buffer full). Moves saved: ");
      } else {
        Serial.print("Macro recording stopped. Moves saved: ");
      }
      Serial.println(recordedMacroCount);
    }
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
  joystickLinearFullStepEqFromStart = 0;
  joystickRotationFullStepEqFromStart = 0;
  stepSize(fullStep, 0);
  stepSize(fullStep, 1);
  currentState = STATE_JOYSTICK_CTRL;
  drawJoystickCtrlScreen();
  drawJoystickDebugReadout();
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
  beginMacroRecording();
}

void PlayMacro() {
  if (macroRecordingActive) {
    Serial.println("Cannot play while recording is active.");
    return;
  }

  if (recordedMacroCount == 0) {
    Serial.println("No recorded macro to play.");
    return;
  }

  Serial.print("Playing macro. Moves: ");
  Serial.println(recordedMacroCount);

  for (uint16_t i = 0; i < recordedMacroCount; i++) {
    if (linearStopRequested) {
      Serial.println("Macro playback stopped by home switch event.");
      break;
    }

    const MacroMove& move = recordedMacro[i];
    stepSize(move.stepMode, move.driver);
    callStep(move.driver, move.direction, move.steps, MACRO_PLAYBACK_PULSE_DELAY_MS);
    delay(MACRO_PLAYBACK_BETWEEN_MOVES_MS);

    int32_t unitsPerPulse = 16;
    switch (move.stepMode) {
      case fullStep: unitsPerPulse = 16; break;
      case halfStep: unitsPerPulse = 8; break;
      case quarterStep: unitsPerPulse = 4; break;
      case eightStep: unitsPerPulse = 2; break;
      case sixteenthStep: unitsPerPulse = 1; break;
      default: unitsPerPulse = 16; break;
    }

    int32_t deltaRaw = (int32_t)move.steps * unitsPerPulse;

    if (move.driver) {
      if (move.direction == 0) {
        joystickLinearFullStepEqFromStart += deltaRaw;
      } else {
        joystickLinearFullStepEqFromStart -= deltaRaw;
      }
    } else {
      uint32_t rotationDeltaAbsRaw = (uint32_t)deltaRaw;
      rotationAbsRemainderRaw += rotationDeltaAbsRaw;
      while (rotationAbsRemainderRaw >= ROTATION_RAW_UNITS_PER_REV) {
        rotationAbsRemainderRaw -= ROTATION_RAW_UNITS_PER_REV;
        rotationTurnCount++;
      }

      if (move.direction == 0) {
        joystickRotationFullStepEqFromStart += deltaRaw;
      } else {
        joystickRotationFullStepEqFromStart -= deltaRaw;
      }
    }
  }

  linearDistanceRaw = joystickLinearFullStepEqFromStart;
  rotationDistanceRaw = joystickRotationFullStepEqFromStart;
  linearDistanceMM = rawLinearToMM(linearDistanceRaw);
  rotationDistance = rawRotationToDegrees(rotationDistanceRaw);

  Serial.println("Macro playback finished.");
}

void setup() {
  pinMode(vrx, INPUT);
  pinMode(vry, INPUT);
  pinMode(jsw, INPUT_PULLUP);
  pinMode(HOMESWITCH, INPUT_PULLUP);

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
  attachInterrupt(digitalPinToInterrupt(HOMESWITCH), onHomeSwitchChange, CHANGE);
  joystickSwitchState = digitalRead(jsw);
  joystickSwitchLastEdgeMs = millis();

  Serial.println("System Ready");
  testMotors();
  Serial.println("Testing Motors");

  
}



void loop() {
  processHomeSwitchEvents();
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


