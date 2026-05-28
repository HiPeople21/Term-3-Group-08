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

// --- Motor speed (manual control) ---
static const int trackSpeed = 800;

// --- Line Following / State Machine ---
float Kp = 1.0;
float Ki = 0.0;
float Kd = 0.0;

const int baseSpeed = 800 * 6 / 7.2;
const int maxSpeed  = 800 * 6 / 7.2;
const int minSpeed  = -(800 * 6 / 7.2);
const int setpoint  = 5500;

const long ticksToHole  = 1355;
const long ticksToPlant = 233;

const unsigned long IR_WINDOW_MS = 1000;

const long junctionCooldownTicks = 200;
long lastJunctionTick = -9999;

float integral  = 0;
float prevError = 0;
unsigned long prevTime = 0;

bool running = false;

int prevMiddleValue = 0;

long encoderAtIR   = 0;
long planterTarget = 0;

unsigned long irDetectedAt     = 0;
unsigned long plantingStartedAt = 0;

enum Stage {
  BASE,
  LINED,
  BLANK,
  RETURNING
};

Stage stage = BASE; 

enum State {
  FOLLOWING,
  WAITING_FOR_RFID,
  DRIVING_TO_HOLE,
  PLANTING,
  JUNCTION_HANDLING,
  OPENING,
  CLOSING,
  WALL_FOLLOWING,
  
};

State state = FOLLOWING;

// -----------------------------------------------------------------------

void revive() {
  // Placeholder — revival logic to be implemented
  Serial.println("Revive");
}

void checkReviveButton() {
  int reading = digitalRead(REVIVE_BUTTON_PIN);
  if (reading != lastReviveState) lastReviveDebounce = millis();
  lastReviveState = reading;

  if ((millis() - lastReviveDebounce) > debounceDelay && reading != reviveBtnState) {
    reviveBtnState = reading;
    if (reviveBtnState == LOW) {  // Button pressed (active-low with pull-up)
      Serial.println("[Revive] Button pressed.");
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

  // Blink red while killed
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
    if (killBtnState == LOW) {  // Button pressed (active-low with pull-up)
      isKilledLocal = !isKilledLocal;
      Serial.print("[Kill Btn] ");
      Serial.println(isKilledLocal ? "KILLED" : "ENABLED");
      if (isKilledLocal) stopTracks();
    }
  }
}

void checkRFID() {
  if (!mfrc522.PICC_IsNewCardPresent() || !mfrc522.PICC_ReadCardSerial()) return;

  Serial.print("[RFID] UID:");
  for (byte i = 0; i < mfrc522.uid.size; i++) {
    Serial.print(mfrc522.uid.uidByte[i] < 0x10 ? " 0" : " ");
    Serial.print(mfrc522.uid.uidByte[i], HEX);
  }
  Serial.println();
  mfrc522.PICC_HaltA();

  Serial.println("[Planter] Rotation triggered.");
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

bool checkForHole() {
  uint8_t midIdx      = getIRSensorCount() / 2;
  int     middleValue = getIRValue(midIdx);

  bool inHole     = middleValue >= 100 && middleValue <= 400;
  bool wasOutside = prevMiddleValue < 100 || prevMiddleValue > 400;

  prevMiddleValue = middleValue;

  return inHole && wasOutside;
}

bool isJunction() {
  readIRPosition();
  uint8_t lastIdx = getIRSensorCount() - 1;
  return (getIRValue(0) > 800 && getIRValue(lastIdx) > 800);
}

void driveStraight(int speed) {
  setLeftTrack(speed);
  setRightTrack(speed);
}

// -----------------------------------------------------------------------

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000);

  // I2C for RFID + motors
  Wire1.begin();

  // RFID reader
  mfrc522.PCD_Init();
  Serial.println("[RFID] Reader ready.");

  // Motor shield (Wire1, address 0x12)
  initMotors();
  Serial.println("[Motors] Ready.");

  // TOF sensors (Serial1 + Serial4) and ultrasonic (pins 44/42)
  // initSensors();
  Serial.println("[Sensors] TOF + Ultrasonic ready.");

  // IR array (QTR 12-sensor, ~10s calibration)
  initIRArray();
  Serial.println("[IR] Ready.");

  // Kill switch LED + button
  // pinMode(LED_RED_PIN,      OUTPUT);
  // pinMode(LED_GREEN_PIN,    OUTPUT);
  // pinMode(KILL_BUTTON_PIN,  INPUT_PULLUP);
  // pinMode(REVIVE_BUTTON_PIN, INPUT_PULLUP);
  // digitalWrite(LED_RED_PIN,   LOW);
  // digitalWrite(LED_GREEN_PIN, HIGH);
  Serial.println("[Kill Switch] Hardware ready.");

  // WiFi kill switch
  initWifi();

  Serial.println("=== SYSTEM ONLINE ===");
  Serial.println("Tracks: W/S/A/D = Fwd/Rev/Left/Right | X = Stop | G = Start autonomy");

  setupGrid();
  register_bot();
}

void loop() {
  // WiFi kill switch (MQTT)
  loopWifi();

  // Mechanical kill switch button
  // checkKillButton();

  // Revival button (pin 48)
  // checkReviveButton();

  // LED reflects combined kill state
  // updateLED();

  // Stop motors whenever system is killed
  bool killed = isKilledLocal || !isSystemEnabled();
  static bool wasPreviouslyKilled = false;
  if (killed && !wasPreviouslyKilled) {
    stopTracks();
    stopPlanter();
    wasPreviouslyKilled = true;
  } else if (!killed) {
    wasPreviouslyKilled = false;
  }

  if (!killed && Serial.available() > 0) {
    char cmd = Serial.read();

    if (cmd == 'g' || cmd == 'G') {
      running = true;
      resetPID();
      state = FOLLOWING;
      Serial.println("Running");
    } else if (cmd == 'x' || cmd == 'X') {
      running = false;
      stopTracks();
      Serial.println("Stopped");
    } else if (!running) {
      if      (cmd == 'w' || cmd == 'W') { Serial.println("Executed w"); setRightTrack(trackSpeed);  setLeftTrack(trackSpeed);  }
      else if (cmd == 's' || cmd == 'S') { Serial.println("Executed s"); setRightTrack(-trackSpeed); setLeftTrack(-trackSpeed); }
      else if (cmd == 'a' || cmd == 'A') { Serial.println("Executed a"); setRightTrack(trackSpeed);  setLeftTrack(-trackSpeed); }
      else if (cmd == 'd' || cmd == 'D') { Serial.println("Executed d"); setRightTrack(-trackSpeed); setLeftTrack(trackSpeed);  }
      else if (cmd == '1')               { Serial.println("Executed 1"); openAirlockA(); }
      else if (cmd == '2')               { Serial.println("Executed 2"); openAirlockB(); }
      else if (cmd == '3') {
        Serial.println("Executed 3");
        int index = Serial.parseInt();
        seedPlanted(UIDs[index]);
      }
      else if (cmd == '4') {
        Serial.println("Executed 4");
        int index = Serial.parseInt();
        checkFertility(UIDs[index]);
      }
    }
  }

  // RFID — triggers planter rotation when card detected (manual mode only)
  // checkRFID();

  if (running && !killed) {
    Serial.println(state);
    switch (stage){
      case BASE:

        break;
      case BLANK:
        break;
      case RETURNING:
        break;

      case LINED:
        switch (state) {

          case FOLLOWING: {
            runLineFollower();

            if (checkForHole()) {
              irDetectedAt = millis();
              encoderAtIR  = getTrackEncoder();
              state = WAITING_FOR_RFID;
              Serial.println("Hole detected");
              break;
            } else if (isJunction()) {
              stopTracks();
              state = JUNCTION_HANDLING;
              Serial.println("Junction detected");
              break;
            } else {
              Serial.print("Following Line, not at junction");
              break;
            }
          }

          case JUNCTION_HANDLING: {
            driveStraight(600 * 6 / 7.2);
            delay(200);
            stopTracks();

            lastJunctionTick = getTrackEncoder();
            resetPID();

            state = FOLLOWING;
            Serial.println("Junction cleared");
            break;
          }

          case WAITING_FOR_RFID: {
            if (isJunction()) {
              driveStraight(600 * 6 / 7.2);
              Serial.println("going straight to rfid");
            } else {
              runLineFollower();
            }

            if (millis() - irDetectedAt > IR_WINDOW_MS) {
              state = FOLLOWING;
              Serial.println("RFID timeout, false positive ir detection");
              break;
            }

            if (mfrc522.PICC_IsNewCardPresent() && mfrc522.PICC_ReadCardSerial()) {
              mfrc522.PICC_HaltA();
              mfrc522.PCD_StopCrypto1();
              state = DRIVING_TO_HOLE;
              Serial.println("RFID confirmed");
            }
            break;
          }

          case DRIVING_TO_HOLE: {
            long currentTicks = getTrackEncoder();
            long tickTarget   = encoderAtIR + ticksToHole;
            long encerror     = tickTarget - currentTicks;

            if (encerror > 5) {
              driveStraight(600 * 6 / 7.2);
              Serial.print("going straight to hole");
            } else {
              stopTracks();
              planterTarget    = getPlanterEncoder() + ticksToPlant;
              plantingStartedAt = millis();
              state = PLANTING;
              Serial.println("At planting position");
            }
            break;
          }

          case PLANTING: {
            long planterPos = getPlanterEncoder();

            if (abs(planterPos - planterTarget) > 5) {
              setPlanter(500 * 6 / 7.2);
            } else {
              setPlanter(0);
              delay(500);
              resetPID();
              state = FOLLOWING;
              Serial.println("Plant complete");
            }
            break;
          }
        }
        break;
    }
  } else if (!killed) {
    // Drive planter motor toward target position (manual mode)
    rotatePlanter();
  }

  // Sensor readings — printed as fast as data arrives (TOF) or every 100ms (ultrasonic/IR)
  // readTOFSensors();
  // readUltrasonic();
  // readIRArray();
}
