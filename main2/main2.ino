#include <Wire.h>
#include <MFRC522_I2C.h>
#include "motors.h"
#include "sensors.h"
#include "wifi_utils.h"

// ===== Hardware Pins =====
#define KILL_BUTTON_PIN   39
#define LED_RED_PIN       38
#define LED_GREEN_PIN     40
#define REVIVE_BUTTON_PIN 48

// ===== RFID =====
MFRC522_I2C mfrc522(0x28, -1, &Wire1);

// ===== Voltage Compensation =====
const float voltageScale = 6.0 / 7.2;

// ===== Line-Following PID =====
float Kp = 2.0, Ki = 0.0, Kd = 0.0;
const int baseSpeed = (int)(800 * voltageScale);
const int maxSpeed  = (int)(850 * voltageScale);
const int minSpeed  = -(int)(800 * voltageScale);
const int setpoint  = 5500;
float integral = 0, prevError = 0;
unsigned long prevTime = 0;

// ===== Wall-Following PID =====
float Kp_wall = 5.0, Ki_wall = 0.0, Kd_wall = 0.0;
const int WALL_TARGET_MM = 70;
const int WALL_BASE_PWM  = 580;
float integral_wall = 0, prevError_wall = 0;
unsigned long lastWallControl = 0;
float smoothedRight = 70.0, smoothedLeft = 70.0;
unsigned long tunnelEnteredAt = 0;
unsigned long wallLostAt = 0;

// ===== Arena & Planting (from ir_array_test) =====
const long ticksToHole  = 1355;
const long ticksToPlant = 233;
const unsigned long IR_WINDOW_MS = 1000;
int seedsPlanted = 0;
const int MAX_SEEDS = 5;
unsigned long arenaEnteredAt = 0;
long encoderAtIR = 0;
unsigned long irDetectedAt = 0;
long planterTarget = 0;
int prevMiddleValue = 0;
String detectedUID = "";

// ===== Junction Tracking =====
long lastJunctionTick = -9999;

// ===== Kill Switch =====
bool isKilledLocal = true;
int killBtnState = HIGH, lastBtnState = HIGH;
unsigned long lastDebounceTime = 0;
const unsigned long debounceDelay = 50;

// ===== Revival Button =====
int reviveBtnState = HIGH, lastReviveState = HIGH;
unsigned long lastReviveDebounce = 0;

// ===== LED =====
unsigned long ledPrevMillis = 0;
const long blinkInterval = 500;
bool redLedOn = false;

// ===== Turn State =====
bool turnActive = false;

// ===== Return Tracking =====
int returnStep = 0;
unsigned long blankStartedAt = 0;
bool airlockBRequested = false;

// ===== Base Exit Tracking =====
bool airlockARequested = false;

// ===== Challenge Stages =====
enum Stage {
  STAGE_BASE_EXIT,
  STAGE_TUNNEL_OUT,
  STAGE_ARENA,
  STAGE_ARENA_RETURN,
  STAGE_TUNNEL_IN,
  STAGE_BASE_PARK,
  STAGE_DONE
};
Stage stage = STAGE_BASE_EXIT;

// ===== Arena Substates (from ir_array_test — DO NOT CHANGE) =====
enum ArenaState {
  FOLLOWING,
  WAITING_FOR_RFID,
  DRIVING_TO_HOLE,
  PLANTING,
  JUNCTION_HANDLING
};
ArenaState arenaState = FOLLOWING;

// ===== Base Exit Substates =====
enum BaseSubState { BS_FOLLOWING, BS_JUNCTION };
BaseSubState baseSubState = BS_FOLLOWING;

// ================================================================
// LINE-FOLLOWING HELPERS
// ================================================================

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
  integral = 0;
  prevError = 0;
  prevTime = millis();
}

void runLineFollower() {
  uint16_t position = readIRPosition();
  float correction = computePID(position);
  int leftSpeed  = constrain(baseSpeed - correction, minSpeed, maxSpeed);
  int rightSpeed = constrain(baseSpeed + correction, minSpeed, maxSpeed);
  setLeftTrack(leftSpeed);
  setRightTrack(rightSpeed);
}

void driveStraight(int speed) {
  setLeftTrack(speed);
  setRightTrack(speed);
}

void angleRight(int speed) {
  setLeftTrack(speed);
  setRightTrack(0);
}

// ================================================================
// SENSOR HELPERS
// ================================================================

bool isJunction() {
  readIRPosition();
  uint8_t lastIdx = getIRSensorCount() - 1;
  return (getIRValue(0) > 800 && getIRValue(lastIdx) > 800);
}

bool isBlank() {
  readIRPosition();
  for (uint8_t i = 0; i < getIRSensorCount(); i++) {
    if (getIRValue(i) < 100) return false;
  }
  return true;
}

bool checkForHole() {
  int middleValue = getIRValue(getIRSensorCount() / 2);
  bool inHole = middleValue >= 100 && middleValue <= 400;
  bool wasOutside = prevMiddleValue < 100 || prevMiddleValue > 400;
  prevMiddleValue = middleValue;
  return inHole && wasOutside;
}

String readRFIDTag() {
  if (!mfrc522.PICC_IsNewCardPresent() || !mfrc522.PICC_ReadCardSerial())
    return "";
  String uid = "";
  for (byte i = 0; i < mfrc522.uid.size; i++) {
    if (mfrc522.uid.uidByte[i] < 0x10) uid += "0";
    uid += String(mfrc522.uid.uidByte[i], HEX);
  }
  uid.toUpperCase();
  mfrc522.PICC_HaltA();
  mfrc522.PCD_StopCrypto1();
  return uid;
}

// ================================================================
// WALL FOLLOWING
// ================================================================

void resetWallFollow() {
  integral_wall = 0;
  prevError_wall = 0;
  lastWallControl = millis();
  smoothedRight = WALL_TARGET_MM;
  smoothedLeft = WALL_TARGET_MM;
  tunnelEnteredAt = 0;
  wallLostAt = 0;
}

bool runWallFollow() {
  readTOFSensors();
  unsigned long now = millis();
  float dt = (now - lastWallControl) / 1000.0;
  if (dt < 0.02) return false;

  unsigned long dR = getTOFRightDist();
  unsigned long dL = getTOFLeftDist();
  bool seeR = isTOFRightFresh() && dR > 0 && dR < 500;
  bool seeL = isTOFLeftFresh()  && dL > 0 && dL < 500;

  if (seeR) smoothedRight = 0.5f * (float)dR + 0.5f * smoothedRight;
  if (seeL) smoothedLeft  = 0.5f * (float)dL + 0.5f * smoothedLeft;

  if ((seeR || seeL) && tunnelEnteredAt == 0) tunnelEnteredAt = millis();

  if (!seeR && !seeL) {
    if (tunnelEnteredAt == 0) {
      driveStraight(WALL_BASE_PWM);
      lastWallControl = now;
      return false;
    }
    if (millis() - tunnelEnteredAt < 2000) {
      driveStraight(WALL_BASE_PWM);
      lastWallControl = now;
      return false;
    }
    if (wallLostAt == 0) wallLostAt = millis();
    if (millis() - wallLostAt > 1000) {
      stopTracks();
      return true;
    }
    driveStraight(WALL_BASE_PWM);
    integral_wall = 0;
    lastWallControl = now;
    return false;
  }

  wallLostAt = 0;

  float error = 0.0;
  bool followRight = true;

  if (seeR && seeL) {
    if (smoothedRight <= smoothedLeft) {
      followRight = true;
      error = smoothedRight - WALL_TARGET_MM;
    } else {
      followRight = false;
      error = smoothedLeft - WALL_TARGET_MM;
    }
  } else if (seeR) {
    followRight = true;
    error = smoothedRight - WALL_TARGET_MM;
  } else {
    followRight = false;
    error = smoothedLeft - WALL_TARGET_MM;
  }

  integral_wall += error * dt;
  integral_wall = constrain(integral_wall, -300.0f, 300.0f);
  float derivative = (error - prevError_wall) / dt;
  float correction = (Kp_wall * error) + (Ki_wall * integral_wall) + (Kd_wall * derivative);
  correction = constrain(correction, -200.0f, 200.0f);

  int cmdL = WALL_BASE_PWM, cmdR = WALL_BASE_PWM;
  if (followRight) { cmdL += (int)correction; cmdR -= (int)correction; }
  else             { cmdL -= (int)correction; cmdR += (int)correction; }
  cmdL = constrain(cmdL, 100, maxSpeed);
  cmdR = constrain(cmdR, 100, maxSpeed);
  setLeftTrack(cmdL);
  setRightTrack(cmdR);

  prevError_wall = error;
  lastWallControl = now;
  return false;
}

// ================================================================
// KILL SWITCH, LED, REVIVAL
// ================================================================

void checkKillButton() {
  int reading = digitalRead(KILL_BUTTON_PIN);
  if (reading != lastBtnState) lastDebounceTime = millis();
  lastBtnState = reading;
  if ((millis() - lastDebounceTime) > debounceDelay && reading != killBtnState) {
    killBtnState = reading;
    if (killBtnState == LOW) {
      isKilledLocal = !isKilledLocal;
      if (isKilledLocal) { stopTracks(); stopPlanter(); }
    }
  }
}

void checkReviveButton() {
  int reading = digitalRead(REVIVE_BUTTON_PIN);
  if (reading != lastReviveState) lastReviveDebounce = millis();
  lastReviveState = reading;
  if ((millis() - lastReviveDebounce) > debounceDelay && reading != reviveBtnState) {
    reviveBtnState = reading;
  }
}

void updateLED() {
  if (digitalRead(REVIVE_BUTTON_PIN) == LOW) {
    digitalWrite(LED_RED_PIN, LOW);
    digitalWrite(LED_GREEN_PIN, HIGH);
    return;
  }
  bool killed = isKilledLocal || (!isSystemEnabled() && !isEmergency());
  if (!killed) {
    digitalWrite(LED_RED_PIN, HIGH);
    digitalWrite(LED_GREEN_PIN, LOW);
    return;
  }
  unsigned long now = millis();
  if (now - ledPrevMillis >= (unsigned long)blinkInterval) {
    ledPrevMillis = now;
    redLedOn = !redLedOn;
    digitalWrite(LED_RED_PIN, redLedOn ? HIGH : LOW);
    digitalWrite(LED_GREEN_PIN, LOW);
  }
}

// ================================================================
// SETUP
// ================================================================

void setup() {
  Wire1.begin();
  mfrc522.PCD_Init();
  initMotors();
  initSensors();
  initIRArray();

  pinMode(LED_RED_PIN,       OUTPUT);
  pinMode(LED_GREEN_PIN,     OUTPUT);
  pinMode(KILL_BUTTON_PIN,   INPUT_PULLUP);
  pinMode(REVIVE_BUTTON_PIN, INPUT_PULLUP);
  digitalWrite(LED_RED_PIN,   HIGH);
  digitalWrite(LED_GREEN_PIN, LOW);

  initWifi();
  register_bot();
  setupGrid();

  prevTime = millis();
}

// ================================================================
// MAIN LOOP
// ================================================================

void loop() {
  loopWifi();
  checkKillButton();
  checkReviveButton();
  updateLED();

  bool killed = isKilledLocal || (!isSystemEnabled() && !isEmergency());

  static bool wasKilled = false;
  if (killed && !wasKilled) {
    stopTracks();
    stopPlanter();
    wasKilled = true;
  } else if (!killed) {
    wasKilled = false;
  }

  if (killed) return;

  if (turnActive) {
    if (updateTurn()) {
      turnActive = false;
    } else {
      return;
    }
  }

  switch (stage) {

    // ============================================================
    // STAGE 1: BASE EXIT
    // ============================================================
    case STAGE_BASE_EXIT: {
      switch (baseSubState) {
        case BS_FOLLOWING: {
          if (airlockARequested) {
            if (blankStartedAt == 0) blankStartedAt = millis();
            unsigned long elapsed = millis() - blankStartedAt;

            if (elapsed < 10500) {
              uint8_t lastIdx = getIRSensorCount() - 1;
              bool sideBranch = (getIRValue(0) > 800 && getIRValue(1) > 800) ||
                                (getIRValue(lastIdx) > 800 && getIRValue(lastIdx - 1) > 800);
              if (isJunction() || sideBranch) {
                angleRight((int)(700 * voltageScale));
                delay(300);
                stopTracks();
                lastJunctionTick = getTrackEncoder();
                resetPID();
              }
              runLineFollower();
            } else if (elapsed < 11000) {
              driveStraight(baseSpeed);
            } else {
              resetWallFollow();
              stage = STAGE_TUNNEL_OUT;
            }
            break;
          }

          runLineFollower();

          String uid = readRFIDTag();
          if (uid.length() > 0) {
            detectedUID = uid;
            openAirlock(uid, 'A');
            airlockARequested = true;
            blankStartedAt = 0;
            break;
          }

          if (isJunction()) {
            stopTracks();
            baseSubState = BS_JUNCTION;
          } else if (isBlank()) {
            driveStraight((int)(500 * voltageScale));
          }
          break;
        }

        case BS_JUNCTION: {
          lastJunctionTick = getTrackEncoder();
          angleRight((int)(700 * voltageScale));
          delay(300);
          resetPID();
          baseSubState = BS_FOLLOWING;
          break;
        }
      }
      break;
    }

    // ============================================================
    // STAGE 2: TUNNEL OUT
    // ============================================================
    case STAGE_TUNNEL_OUT: {
      digitalWrite(LED_GREEN_PIN, HIGH);
      if (runWallFollow()) {
        arenaEnteredAt = millis();
        clearEmergency();
        stage = STAGE_ARENA;
        arenaState = FOLLOWING;
        resetPID();
        Kp = 2.0, Ki = 0.0, Kd = 0.0;
      }
      break;
    }

    // ============================================================
    // STAGE 3: ARENA — ir_array_test logic, untouched
    //
    // Line follow. IR detects hole → scan RFID while driving
    // ticksToHole → plant. Junctions: drive straight through.
    // ============================================================
    case STAGE_ARENA: {

      switch (arenaState) {

        case FOLLOWING: {
          runLineFollower();
          if (checkForHole()) {
            irDetectedAt = millis();
            encoderAtIR  = getTrackEncoder();
            arenaState = WAITING_FOR_RFID;
          } else if (isJunction()) {
            stopTracks();
            arenaState = JUNCTION_HANDLING;
          }
          break;
        }

        case JUNCTION_HANDLING: {
          driveStraight(600 * voltageScale);
          delay(200);
          stopTracks();
          lastJunctionTick = getTrackEncoder();
          resetPID();
          arenaState = FOLLOWING;
          break;
        }

        case WAITING_FOR_RFID: {
          if (isJunction()) {
            driveStraight(600 * voltageScale);
          } else {
            runLineFollower();
          }

          if (millis() - irDetectedAt > IR_WINDOW_MS) {
            arenaState = FOLLOWING;
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
            checkFertility(detectedUID);
            arenaState = DRIVING_TO_HOLE;
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
            arenaState = PLANTING;
          }
          break;
        }

        case PLANTING: {
          long planterPos = getPlanterEncoder();

          if (abs(planterPos - planterTarget) > 5) {
            setPlanter(500 * voltageScale);
          } else {
            setPlanter(0);
            seedsPlanted++;
            if (detectedUID.length() > 0) seedPlanted(detectedUID);
            delay(500);
            resetPID();
            arenaState = FOLLOWING;
          }
          break;
        }
      }
      break;
    }

    // ============================================================
    // STAGE 4: ARENA RETURN
    // ============================================================
    case STAGE_ARENA_RETURN: {
      switch (returnStep) {
        case 0: {
          startTurn(180.0);
          turnActive = true;
          returnStep = 1;
          break;
        }
        case 1: {
          resetPID();
          blankStartedAt = 0;
          returnStep = 2;
          break;
        }
        case 2: {
          if (isBlank()) {
            if (blankStartedAt == 0) blankStartedAt = millis();
            if (millis() - blankStartedAt > 1500) {
              stopTracks();
              returnStep = 3;
            } else {
              driveStraight(baseSpeed);
            }
          } else {
            blankStartedAt = 0;
            if (isJunction()) {
              driveStraight(baseSpeed);
              delay(200);
            } else {
              runLineFollower();
            }
          }

          if (!airlockBRequested) {
            String uid = readRFIDTag();
            if (uid.length() > 0) {
              openAirlock(uid, 'B');
              airlockBRequested = true;
            }
          }
          break;
        }
        case 3: {
          startTurn(-90.0);
          turnActive = true;
          returnStep = 4;
          break;
        }
        case 4: {
          returnStep = 5;
          break;
        }
        case 5: {
          if (!airlockBRequested) {
            String uid = readRFIDTag();
            if (uid.length() > 0) {
              openAirlock(uid, 'B');
              airlockBRequested = true;
            }
          }

          driveStraight(baseSpeed);

          readTOFSensors();
          unsigned long dR = getTOFRightDist();
          unsigned long dL = getTOFLeftDist();
          bool seeR = isTOFRightFresh() && dR > 0 && dR < 500;
          bool seeL = isTOFLeftFresh()  && dL > 0 && dL < 500;

          if (seeR || seeL) {
            resetWallFollow();
            stage = STAGE_TUNNEL_IN;
          }
          break;
        }
      }
      break;
    }

    // ============================================================
    // STAGE 5: TUNNEL IN
    // ============================================================
    case STAGE_TUNNEL_IN: {
      if (runWallFollow()) {
        clearEmergency();
        stage = STAGE_BASE_PARK;
        blankStartedAt = millis();
        resetPID();
      }
      break;
    }

    // ============================================================
    // STAGE 6: BASE PARK
    // ============================================================
    case STAGE_BASE_PARK: {
      if (isBlank()) {
        if (blankStartedAt == 0) blankStartedAt = millis();
        if (millis() - blankStartedAt > 3000) {
          stopTracks();
          stage = STAGE_DONE;
        } else {
          driveStraight(baseSpeed);
        }
      } else {
        blankStartedAt = 0;
        if (isJunction()) {
          angleRight((int)(500 * voltageScale));
          delay(200);
          resetPID();
        } else {
          runLineFollower();
        }
      }

      String uid = readRFIDTag();
      if (uid.length() > 0) {
        openAirlock(uid, 'B');
      }
      break;
    }

    // ============================================================
    // STAGE 7: DONE
    // ============================================================
    case STAGE_DONE: {
      stopTracks();
      stopPlanter();
      break;
    }
  }
}
