#include <Wire.h>
#include <MFRC522_I2C.h>
#include "motors.h"
#include "sensors.h"
#include "wifi_utils.h"

// RFID setup (Wire1, I2C addr 0x28)
MFRC522_I2C mfrc522(0x28, -1, &Wire1);

// Kill switch pins
#define KILL_BUTTON_PIN 39
#define LED_RED_PIN     38
#define LED_GREEN_PIN   40

// Kill switch states
static bool isKilledLocal    = false;
static int  killBtnState     = HIGH;
static int  lastBtnState     = HIGH;
static unsigned long lastDebounceTime = 0;
static const unsigned long debounceDelay = 50;

// LED blink states
static unsigned long previousMillis = 0;
static const long blinkInterval = 500;
static bool redLedOn = false;

// Revive button pins & states
#define REVIVE_BUTTON_PIN 48
static int  reviveBtnState  = HIGH;
static int  lastReviveState = HIGH;
static unsigned long lastReviveDebounce = 0;

// Motor config
const int baseSpeed = 500 * 6 / 7.2;

bool running = true;

// Task 4 Hardcoded Vars
int t4Step = 0;        
long startTicks = 0;   

// TODO: Update this value after testing real distance
const long TICKS_PER_NODE = 2700; 

enum Stage {
  TASK_4_OPEN_FIELD,
  CALIBRATION, 
  DONE
};

Stage stage = TASK_4_OPEN_FIELD; 

enum State {
  FOLLOWING,
  TURNING
};

State turnReturnState = FOLLOWING;
State state = FOLLOWING;

void revive() {
  // Placeholder for revive logic
}

void checkReviveButton() {
  int reading = digitalRead(REVIVE_BUTTON_PIN);
  if (reading != lastReviveState) lastReviveDebounce = millis();
  lastReviveState = reading;
  
  if ((millis() - lastReviveDebounce) > debounceDelay && reading != reviveBtnState) {
    reviveBtnState = reading;
    if (reviveBtnState == LOW) revive();
  }
}

void updateLED() {
  bool killed = isKilledLocal || !isSystemEnabled();
  
  if (!killed) {
    digitalWrite(LED_RED_PIN,   LOW);
    digitalWrite(LED_GREEN_PIN, HIGH);
    redLedOn = false;
    return;
  }
  
  // Blink red LED if killed
  unsigned long now = millis();
  if (now - previousMillis >= (unsigned long)blinkInterval) {
    previousMillis = now;
    redLedOn = !redLedOn;
    digitalWrite(LED_RED_PIN,   redLedOn ? HIGH : LOW);
    digitalWrite(LED_GREEN_PIN, LOW);
  }
}

void checkKillButton() {
  int reading = digitalRead(KILL_BUTTON_PIN);
  if (reading != lastBtnState) lastDebounceTime = millis();
  lastBtnState = reading;
  
  if ((millis() - lastDebounceTime) > debounceDelay && reading != killBtnState) {
    killBtnState = reading;
    if (killBtnState == LOW) {
      isKilledLocal = !isKilledLocal;
      if (isKilledLocal) stopTracks();
    }
  }
}

void driveStraight(int speed) {
  setLeftTrack(speed);
  setRightTrack(speed);
}

void initTurn(float degrees, State returnState) {
  turnReturnState = returnState;
  startTurn(degrees);
  state = TURNING;
}

void setup() {
  Serial.begin(115200);
  
  Wire1.begin();
  mfrc522.PCD_Init();
  initMotors();
  initSensors();
  
  // Skipped initIRArray() for faster boot

  pinMode(LED_RED_PIN,       OUTPUT);
  pinMode(LED_GREEN_PIN,     OUTPUT);
  pinMode(KILL_BUTTON_PIN,   INPUT_PULLUP);
  pinMode(REVIVE_BUTTON_PIN, INPUT_PULLUP);
  digitalWrite(LED_RED_PIN,   LOW);
  digitalWrite(LED_GREEN_PIN, HIGH);

  initWifi();
  setupGrid();
  register_bot();

  // Mode switch: Currently in CALIBRATION
  // Change stage to TASK_4_OPEN_FIELD once calibration is done
  stage = CALIBRATION; 
  state = FOLLOWING;
  t4Step = 0;
}

void loop() {
  loopWifi();
  checkKillButton();
  checkReviveButton();
  updateLED();

  bool killed = isKilledLocal || !isSystemEnabled();
  static bool wasPreviouslyKilled = false;
  
  if (killed && !wasPreviouslyKilled) {
    stopTracks();
    stopPlanter();
    wasPreviouslyKilled = true;
  } else if (!killed) {
    wasPreviouslyKilled = false;
  }

  // Handle turning logic
  if (!killed && state == TURNING) {
    if (updateTurn()) {
      state = turnReturnState;
    }
  }

  if (running && !killed) {
    switch (stage){
      
      // Calibration Mode
      case CALIBRATION: {
        stopTracks(); // Force stop for safety
        
        static unsigned long lastPrintTime = 0;
        if (millis() - lastPrintTime > 200) {
          lastPrintTime = millis();
          long currentTicks = getTrackEncoder();
          Serial.print("[Calibration] Current encoder ticks: ");
          Serial.println(currentTicks);
        }
        break;
      }

      // Task 4: Open-Field Dead Reckoning
      case TASK_4_OPEN_FIELD: {
        
        // Read RFID to avoid I2C bus jam
        if (mfrc522.PICC_IsNewCardPresent() && mfrc522.PICC_ReadCardSerial()) {
          mfrc522.PICC_HaltA();
          mfrc522.PCD_StopCrypto1();
        }

        if (state == TURNING) break;

        long currentTicks = getTrackEncoder();

        switch (t4Step) {
          case 0: // Init and go straight (target: 2 nodes)
            startTicks = currentTicks;
            driveStraight(baseSpeed);
            t4Step = 1;
            break;

          case 1: // Wait until 2 nodes reached
            if (abs(currentTicks - startTicks) >= (2 * TICKS_PER_NODE)) {
              stopTracks();
              delay(200); 
              initTurn(90.0, FOLLOWING); // Turn right
              t4Step = 2;
            }
            break;

          case 2: // Turn done, go straight (target: 1 node)
            if (state == FOLLOWING) {
              startTicks = currentTicks;
              driveStraight(baseSpeed);
              t4Step = 3;
            }
            break;

          case 3: // Wait until 1 node reached
            if (abs(currentTicks - startTicks) >= (1 * TICKS_PER_NODE)) {
              stopTracks();
              delay(200);
              initTurn(-90.0, FOLLOWING); // Turn left
              t4Step = 4;
            }
            break;

          case 4: // Turn done, go straight (target: 2 nodes)
            if (state == FOLLOWING) {
              startTicks = currentTicks;
              driveStraight(baseSpeed);
              t4Step = 5;
            }
            break;

          case 5: // Wait for final 2 nodes
            if (abs(currentTicks - startTicks) >= (2 * TICKS_PER_NODE)) {
              stopTracks();
              stage = DONE; // Mission accomplished
            }
            break;
        }
        break;
      }

      case DONE: {
        stopTracks();
        break;
      }
    }
  } else if (!killed) {
    // Manual mode planter code if needed
  }
}
