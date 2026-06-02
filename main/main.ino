#include <Wire.h>
#include <MFRC522_I2C.h>
#include "motors.h"
#include "sensors.h"
#include "wifi_utils.h"

// --- RFID (Wire1, I2C address 0x28) ---
MFRC522_I2C mfrc522(0x28, -1, &Wire1);

// --- Kill Switch Pins ---
#define KILL_BUTTON_PIN 39
#define LED_RED_PIN     38
#define LED_GREEN_PIN   40

// --- Kill Switch State ---
static bool isKilledLocal    = false;
static int  killBtnState     = HIGH;
static int  lastBtnState     = HIGH;
static unsigned long lastDebounceTime = 0;
static const unsigned long debounceDelay = 50;

// --- LED Blink State ---
static unsigned long previousMillis = 0;
static const long blinkInterval = 500;
static bool redLedOn = false;

// --- Revival Button (pin 48) ---
#define REVIVE_BUTTON_PIN 48
static int  reviveBtnState  = HIGH;
static int  lastReviveState = HIGH;
static unsigned long lastReviveDebounce = 0;

// --- Voltage compensation: 6V motor rating / 7.2V battery ---
const float voltageScale = 6.0 / 7.2;

// --- Motor speed (manual control) ---
static const int trackSpeed = 800 * voltageScale;

// --- Line Following / State Machine ---
float Kp = 1.15;
float Ki = 0.0;
float Kd = 0.0;

const int baseSpeed = 800 * voltageScale;
const int maxSpeed  = 800 * voltageScale;
const int minSpeed  = -(800 * voltageScale);
const int setpoint  = 5500;

const long ticksToHole  = 1355;
const long ticksToPlant = 233;

const unsigned long IR_WINDOW_MS = 1000;
const unsigned long FERTILITY_TIMEOUT_MS = 5000;

const long junctionCooldownTicks = 200;
long lastJunctionTick = -9999;

float integral  = 0;
float prevError = 0;
unsigned long prevTime = 0;

bool running = true;

int prevMiddleValue = 0;
int turnInBase = 0;

long encoderAtIR   = 0;
long planterTarget = 0;


unsigned long irDetectedAt        = 0;
unsigned long plantingStartedAt   = 0;
unsigned long fertilityRequestedAt = 0;

String detectedUID = "";

enum Stage {
  BASE,
  LINED,
  BLANK,
  RETURNING
};

Stage stage = LINED;

enum State {
  FOLLOWING,
  WAITING_FOR_RFID,
  DRIVING_TO_HOLE,
  PLANTING,
  JUNCTION_HANDLING,
  OPENING,
  CLOSING,
  WALL_FOLLOWING,
  WAITING_FOR_FERTILITY,
  TURNING
};

State turnReturnState = FOLLOWING;
State state = FOLLOWING;

// -----------------------------------------------------------------------

void revive() {
}

void checkReviveButton() {
  int reading = digitalRead(REVIVE_BUTTON_PIN);
  if (reading != lastReviveState) lastReviveDebounce = millis();
  lastReviveState = reading;

  if ((millis() - lastReviveDebounce) > debounceDelay && reading != reviveBtnState) {
    reviveBtnState = reading;
    if (reviveBtnState == LOW) {
      revive();
    }
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

void checkRFID() {
  if (!mfrc522.PICC_IsNewCardPresent() || !mfrc522.PICC_ReadCardSerial()) return;

  for (byte i = 0; i < mfrc522.uid.size; i++) {
  }
  mfrc522.PICC_HaltA();

  triggerPlanterRotation();
}

// --- Line Following Helpers ---

float computePID(float position) {
  unsigned long now = millis();
  float dt = (now - prevTime) / 1000.0;
  if (dt <= 0) return 0;
  prevTime = now;

  float error = setpoint - position;
  integral += error * dt;
  float derivative = (error - prevError) / dt;
  prevError = error;

  return (Kp * error) + (Ki * integral) + (Kd * derivative);
}

void resetPID() {
  integral  = 0;
  prevError = 0;
  prevTime  = millis();
}

void runLineFollower() {
  uint16_t position = readIRPosition();
  float correction  = computePID(position);

  int leftSpeed  = constrain(baseSpeed - correction, minSpeed, maxSpeed);
  int rightSpeed = constrain(baseSpeed + correction, minSpeed, maxSpeed);

  setLeftTrack(leftSpeed);
  setRightTrack(rightSpeed);
}


void angleLeft(int speed){
  setLeftTrack(0);
  setRightTrack(speed);
}

void angleRight(int speed){
  setLeftTrack(speed);
  setRightTrack(0);
}


bool checkForHole() {
  uint8_t midIdx      = getIRSensorCount() / 2;
  int     middleValue = getIRValue(midIdx);

  bool inHole = middleValue >= 100 && middleValue <= 400;
  bool wasOutside = prevMiddleValue < 100 || prevMiddleValue > 400;

  prevMiddleValue = middleValue;

  return inHole && wasOutside;
}

bool isJunction() {
  readIRPosition();
  uint8_t lastIdx = getIRSensorCount()-1;
  return (getIRValue(0) > 800 && getIRValue(lastIdx) > 800);
}

bool isBlank(){
  readIRPosition();
  for (uint8_t i = 0; i < getIRSensorCount(); i++) {
    if (getIRValue(i) < 100) return false;
  }
  return true;
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

// -----------------------------------------------------------------------

void setup() {
  Wire1.begin();

  mfrc522.PCD_Init();

  initMotors();

  initSensors();

  initIRArray();

  // Kill switch LED + button
  pinMode(LED_RED_PIN,      OUTPUT);
  pinMode(LED_GREEN_PIN,    OUTPUT);
  pinMode(KILL_BUTTON_PIN,  INPUT_PULLUP);
  pinMode(REVIVE_BUTTON_PIN, INPUT_PULLUP);
  digitalWrite(LED_RED_PIN,   LOW);
  digitalWrite(LED_GREEN_PIN, HIGH);

  initWifi();

  setupGrid();
  register_bot();

  setFertilityCallback([](bool fertile) {
    if (state == WAITING_FOR_FERTILITY) {
      if (fertile) state = DRIVING_TO_HOLE;
      else state = FOLLOWING;
    }
  });
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

  if (!killed && state == TURNING) {
    if (updateTurn()) {
      state = turnReturnState;
    }
  }

  if (running && !killed) {
    switch (stage){
      
      case BASE: {
        switch(state){
          case FOLLOWING: {
            runLineFollower();
            if (mfrc522.PICC_IsNewCardPresent() && mfrc522.PICC_ReadCardSerial()) {
              detectedUID = "";
              for (byte i = 0; i < mfrc522.uid.size; i++) {
                if (mfrc522.uid.uidByte[i] < 0x10) detectedUID += "0";
                detectedUID += String(mfrc522.uid.uidByte[i], HEX);
              }
              detectedUID.toUpperCase();

              mfrc522.PICC_HaltA();
              mfrc522.PCD_StopCrypto1();
              stopTracks();
              delay(200);
              
              openAirlock(detectedUID, 'A');
            }
            if (isJunction()) {
              stopTracks();
              state = JUNCTION_HANDLING;

            } else if(isBlank()){
              driveStraight(500);
            }

            break;
          }
          case JUNCTION_HANDLING: {
            angleRight(500);
            delay(200);
            break;
          }

        }

        break;
        case BLANK:
          // driveStraight(500);
          // if(!isBlank()){
            //;
            state = FOLLOWING;
          // }
          break;
        case RETURNING:
          break;
      }
      case LINED: {
        switch (state) {

          case FOLLOWING: {
            runLineFollower();
            if (checkForHole()) {
              irDetectedAt = millis();
              encoderAtIR  = getTrackEncoder();
              state = WAITING_FOR_RFID;
            } else if (isJunction()) {
              stopTracks();
              state = JUNCTION_HANDLING;
            }
            break;
          }

          case JUNCTION_HANDLING: {
            driveStraight(600 * voltageScale);
            delay(200);
            stopTracks();

            lastJunctionTick = getTrackEncoder();
            resetPID();

            state = FOLLOWING;
            break;
          }

          case WAITING_FOR_RFID: {
            if (isJunction()) {
              driveStraight(600 * voltageScale);
            } else {
              runLineFollower();
            }

            if (millis() - irDetectedAt > IR_WINDOW_MS) {
              state = FOLLOWING;
              break;
            }

            if (mfrc522.PICC_IsNewCardPresent() && mfrc522.PICC_ReadCardSerial()) {
              detectedUID = "";
              for (byte i = 0; i < mfrc522.uid.size; i++) {
                if (mfrc522.uid.uidByte[i] < 0x10) detectedUID += "0";
                detectedUID += String(mfrc522.uid.uidByte[i], HEX);
              }
              detectedUID.toUpperCase();

              mfrc522.PICC_HaltA();
              mfrc522.PCD_StopCrypto1();
              stopTracks();

              checkFertility(detectedUID);
              fertilityRequestedAt = millis();
              state = WAITING_FOR_FERTILITY;
            }
            break;
          }

          case DRIVING_TO_HOLE: {
            long currentTicks = getTrackEncoder();
            long tickTarget   = encoderAtIR + ticksToHole;
            long encerror     = tickTarget - currentTicks;

            if (encerror > 5) {
              driveStraight(600 * voltageScale);
            } else {
              stopTracks();
              planterTarget    = getPlanterEncoder() + ticksToPlant;
              plantingStartedAt = millis();
              state = PLANTING;
            }
            break;
          }

          case PLANTING: {
            long planterPos = getPlanterEncoder();

            if (abs(planterPos - planterTarget) > 5) {
              setPlanter(500 * voltageScale);
            } else {
              setPlanter(0);
              delay(500);
              resetPID();
              state = FOLLOWING;
            }
            break;
          }

          case WAITING_FOR_FERTILITY: {
            stopTracks();
            if (millis() - fertilityRequestedAt > FERTILITY_TIMEOUT_MS) {
              state = FOLLOWING;
            }
            break;
          }

        }
        break;
      }
      break;
    }
  } else if (!killed) {
    rotatePlanter();
  }
}
