// ============================================================
// main.ino — Competition sketch for Year 1 Robotics Challenge 2026
//
// Full challenge flow:
//   1. BASE_EXIT:    Line follow from deployment → RFID tag → openAirlock A (exit)
//   2. TUNNEL_OUT:   Wall follow through Airlock A tunnel into arena
//   3. ARENA:        Navigate lined grid, read RFID tags, check fertility, plant seeds
//   4. ARENA_RETURN: Turn 180°, line follow back, scan re-entry RFID → openAirlock B
//   5. TUNNEL_IN:    Wall follow through Airlock B tunnel back to base
//   6. BASE_PARK:    Line follow to parking area
//
// Airlock A = exit from base.  Airlock B = reentry into base.
//
// Kill switch (pin 39) toggles start/stop.
// Revival button (pin 48) lights green LED when pressed.
// Emergency warnings trigger immediate return to base.
// ============================================================

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

// ===== Voltage Compensation (6V motors on 7.2V battery) =====
const float voltageScale = 6.0 / 7.2;

// ===== Line-Following PID =====
float Kp = 2.0, Ki = 0.0, Kd = 0.0;
const int baseSpeed = (int)(800 * voltageScale);
const int maxSpeed  = (int)(850 * voltageScale);
const int minSpeed  = -(int)(800 * voltageScale);
const int setpoint  = 5500;
float integral = 0, prevError = 0;
unsigned long prevTime = 0;

// ===== Wall-Following PID (tunnels) — from wallfollowing_2 =====
// float Kp_wall = 2.0, Ki_wall = 0.0, Kd_wall = 7.0;
float Kp_wall = 3, Ki_wall = 0.0, Kd_wall = 0.0;
const int WALL_TARGET_MM = 70;
const int WALL_BASE_PWM  = 580;
const int WALL_DEADBAND  = 60;
float integral_wall = 0, prevError_wall = 0;
unsigned long lastWallControl = 0;
float smoothedRight = 70.0, smoothedLeft = 70.0;
unsigned long tunnelEnteredAt = 0;
unsigned long wallLostAt = 0;

// ===== Arena & Planting =====
const long ticksToHole  = 1355;
const long ticksToPlant = 233;
const unsigned long IR_WINDOW_MS         = 1000;
const unsigned long FERTILITY_TIMEOUT_MS = 5000;
const unsigned long ARENA_TIME_MS        = 240000; // fallback: 4 min if server time unavailable
const int RETURN_BUFFER_SECS             = 60;     // start returning with 60s left

int seedsPlanted = 0;
const int MAX_SEEDS = 5;
unsigned long arenaEnteredAt = 0;
long encoderAtIR = 0;
unsigned long irDetectedAt = 0;
unsigned long fertilityRequestedAt = 0;
long planterTarget = 0;
int prevMiddleValue = 0;
String detectedUID = "";

// ===== Junction Tracking =====
const long junctionCooldownTicks = 200;
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
int baseJunctionCount = 0;
bool airlockARequested = false;

// ===== Arena Grid Navigation =====
const float TICKS_PER_MM = 1400.0 / (38.5 * PI);
const long NODE_SPACING_TICKS = (long)(250.0 * TICKS_PER_MM);
const int UNLINED_ROWS = 5;

int nodesInColumn = 0;
int columnsDone = 0;
int gridDir = 1;           // +1 or -1, alternates each column
int uTurnStep = 0;
long driveStartTicks = 0;
bool onLinedHalf = true;

// Server-confirmed grid position (from isFertileReply)
int gridX = 0, gridY = 0;

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
// Stage stage = STAGE_TUNNEL_OUT;

// ===== Substates =====
enum SubState {
  SS_FOLLOWING,
  SS_JUNCTION,
  SS_WAIT_RFID,
  SS_WAIT_FERTILITY,
  SS_DRIVE_TO_HOLE,
  SS_PLANTING,
  SS_FIND_LINE,
  SS_COLUMN_END,    // U-turn at end of column
  SS_DEAD_RECKON,   // Encoder-counted drive to next node (unlined half)
  SS_DR_ARRIVE      // At dead-reckoned position, scanning RFID
};
SubState subState = SS_FOLLOWING;

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
  uint8_t midIdx = getIRSensorCount() / 2;
  int middleValue = getIRValue(midIdx);
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
// ARENA TIMER / SEED COUNT
// ================================================================

bool shouldReturnToBase() {
  if (seedsPlanted >= MAX_SEEDS) return true;
  if (isEmergency()) return true;
  // Prefer server-provided time_left over manual timer
  int timeLeft = getTimeLeft();
  if (timeLeft >= 0 && timeLeft <= RETURN_BUFFER_SECS) return true;
  // Fallback if server hasn't sent time_left yet
  if (timeLeft < 0 && arenaEnteredAt > 0 && (millis() - arenaEnteredAt) > ARENA_TIME_MS) return true;
  return false;
}

// ================================================================
// WALL FOLLOWING (tunnels)
// Returns true when tunnel exit detected (walls gone for > 500ms)
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

  // Track whether we've ever seen walls (for tunnel exit detection)
  if ((seeR || seeL) && tunnelEnteredAt == 0) tunnelEnteredAt = millis();

  if (!seeR && !seeL) {
    // No walls seen at all
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

  // Dynamic wall selection (from wallfollowing_2) — follow the closer wall
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

  // Clamp correction to prevent violent turns when entering at an angle
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
  // Revival: button pressed → green LED (challenge spec: red default, green on press)
  if (digitalRead(REVIVE_BUTTON_PIN) == LOW) {
    digitalWrite(LED_RED_PIN, LOW);
    digitalWrite(LED_GREEN_PIN, HIGH);
    return;
  }

  bool killed = isKilledLocal || (!isSystemEnabled() && !isEmergency());
  if (!killed) {
    // Running: red LED on (challenge spec: default red)
    digitalWrite(LED_RED_PIN, HIGH);
    digitalWrite(LED_GREEN_PIN, LOW);
    return;
  }
  // Killed/disabled: blink red
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
  Serial.begin(115200);
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

  setFertilityCallback([](bool canPlant, int x, int y) {
    if (subState != SS_WAIT_FERTILITY) return;

    // Store server-confirmed position
    if (x >= 1 && x <= 9 && y >= 1 && y <= 9) {
      gridX = x;
      gridY = y;
      onLinedHalf = (y > UNLINED_ROWS);
    }

    if (canPlant) {
      subState = SS_DRIVE_TO_HOLE;
    } else if (onLinedHalf) {
      subState = SS_FOLLOWING;
      blankStartedAt = 0;
      resetPID();
    } else {
      if (nodesInColumn >= 9) {
        subState = SS_COLUMN_END;
        uTurnStep = 0;
      } else if (y >= UNLINED_ROWS) {
        subState = SS_FIND_LINE;
        blankStartedAt = millis();
      } else {
        driveStartTicks = getTrackEncoder();
        subState = SS_DEAD_RECKON;
      }
    }
  });
}

// ================================================================
// MAIN LOOP
// ================================================================

void loop() {
  loopWifi();
  checkKillButton();
  checkReviveButton();
  updateLED();

  // During emergency the robot must actively return, not just stop
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

  // Global turn handler — blocks stage logic until turn completes
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
    // Base layout: Start → Junction 1 (branches L/R) →
    //   Right path: two left curves with RFID tag between them →
    //   Junction 2 → straight to Airlock A door → tunnel.
    // Turn RIGHT at both junctions.  RFID is read while moving.
    // ============================================================
    case STAGE_BASE_EXIT: {
      switch (subState) {
        case SS_FOLLOWING: {
          // --- After RFID: line follow 10s (wait for door), straight 2s, wall follow ---
          if (airlockARequested) {
            if (blankStartedAt == 0) blankStartedAt = millis();
            unsigned long elapsed = millis() - blankStartedAt;

            if (elapsed < 10500) {
              // Handle J2: angleRight at junctions/side branches
              uint8_t lastIdx = getIRSensorCount() - 1;
              bool sideBranch = (getIRValue(0) > 800 && getIRValue(1) > 800) ||
                                (getIRValue(lastIdx) > 800 && getIRValue(lastIdx - 1) > 800);
              if (isJunction() || sideBranch) {
                stopTracks();
                startTurn(90);
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

          // --- Before RFID: line follow + scan + handle junctions ---
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
            subState = SS_JUNCTION;
          } else if (isBlank()) {
            driveStraight((int)(500 * voltageScale));
          }
          break;
        }

        case SS_JUNCTION: {
          lastJunctionTick = getTrackEncoder();
          angleRight((int)(700 * voltageScale));
          delay(300);
          resetPID();
          subState = SS_FOLLOWING;
          break;
        }

        default: break;
      }
      break;
    }

    // ============================================================
    // STAGE 2: TUNNEL OUT
    // Wall follow through Airlock A tunnel (with ramp) into arena.
    // ============================================================
    case STAGE_TUNNEL_OUT: {
      digitalWrite(LED_GREEN_PIN, HIGH);

      if (runWallFollow()) {
        arenaEnteredAt = millis();
        clearEmergency();
        stage = STAGE_ARENA;
        subState = SS_FIND_LINE;
        blankStartedAt = millis();
        resetPID();
      }
      break;
    }

    // ============================================================
    // STAGE 3: ARENA
    // Boustrophedon scan: follow columns on the lined half (line
    // following), then dead-reckon columns on the unlined half.
    // At each node: read RFID → check fertility → plant if ok.
    // U-turn at end of each column to reach the next one.
    // ============================================================
    case STAGE_ARENA: {
      // Emergency: abort all arena activity and return immediately
      if (isEmergency()) {
        stopTracks();
        stopPlanter();
        // stage = STAGE_ARENA_RETURN;
        returnStep = 0;
        turnActive = false;
        blankStartedAt = 0;
        airlockBRequested = false;
        break;
      }

      // Timer/seed limit: switch to return when between actions
      if (shouldReturnToBase() &&
          (subState == SS_FOLLOWING || subState == SS_DEAD_RECKON ||
           subState == SS_DR_ARRIVE || subState == SS_FIND_LINE)) {
        stopTracks();
        stage = STAGE_ARENA_RETURN;
        returnStep = 0;
        turnActive = false;
        blankStartedAt = 0;
        airlockBRequested = false;
        break;
      }

      switch (subState) {

        // ------ Find a line after tunnel exit or column transition ------
        case SS_FIND_LINE: {
          driveStraight(baseSpeed);
          if (!isBlank()) {
            onLinedHalf = true;
            blankStartedAt = 0;
            subState = SS_FOLLOWING;
            resetPID();
          } else if (millis() - blankStartedAt > 5000) {
            onLinedHalf = false;
            driveStartTicks = getTrackEncoder();
            subState = SS_DEAD_RECKON;
          }
          break;
        }

        // ------ Lined half: PID line follow along a column ------
        case SS_FOLLOWING: {
          runLineFollower();

          // RFID as primary node detector — every hole has a tag
          if (mfrc522.PICC_IsNewCardPresent() && mfrc522.PICC_ReadCardSerial()) {
            detectedUID = "";
            for (byte i = 0; i < mfrc522.uid.size; i++) {
              if (mfrc522.uid.uidByte[i] < 0x10) detectedUID += "0";
              detectedUID += String(mfrc522.uid.uidByte[i], HEX);
            }
            detectedUID.toUpperCase();
            mfrc522.PICC_HaltA();
            mfrc522.PCD_StopCrypto1();

            nodesInColumn++;
            encoderAtIR = getTrackEncoder();
            stopTracks();
            checkFertility(detectedUID);
            fertilityRequestedAt = millis();
            subState = SS_WAIT_FERTILITY;
            break;
          }

          // IR fallback — detect hole, then scan RFID in SS_WAIT_RFID
          if (checkForHole()) {
            irDetectedAt = millis();
            encoderAtIR = getTrackEncoder();
            subState = SS_WAIT_RFID;
            break;
          }

          if (isJunction()) {
            stopTracks();
            subState = SS_JUNCTION;
            break;
          }

          // Sustained blank after visiting nodes — lines ended or column done
          if (nodesInColumn > 0 && isBlank()) {
            if (blankStartedAt == 0) blankStartedAt = millis();
            if (millis() - blankStartedAt > 800) {
              stopTracks();
              if (nodesInColumn >= 9) {
                subState = SS_COLUMN_END;
                uTurnStep = 0;
              } else {
                onLinedHalf = false;
                driveStartTicks = getTrackEncoder();
                subState = SS_DEAD_RECKON;
              }
            }
          } else if (!isBlank()) {
            blankStartedAt = 0;
          }
          break;
        }

        // ------ Junction: drive through and resume ------
        case SS_JUNCTION: {
          driveStraight((int)(600 * voltageScale));
          delay(200);
          stopTracks();
          lastJunctionTick = getTrackEncoder();
          resetPID();
          subState = SS_FOLLOWING;
          break;
        }

        // ------ Hole detected on lined half, scan RFID ------
        case SS_WAIT_RFID: {
          if (isJunction()) {
            driveStraight((int)(600 * voltageScale));
          } else {
            runLineFollower();
          }

          if (millis() - irDetectedAt > IR_WINDOW_MS) {
            subState = SS_FOLLOWING;
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
            nodesInColumn++;
            stopTracks();
            checkFertility(detectedUID);
            fertilityRequestedAt = millis();
            subState = SS_WAIT_FERTILITY;
          }
          break;
        }

        // ------ Waiting for server fertility reply ------
        case SS_WAIT_FERTILITY: {
          stopTracks();
          if (millis() - fertilityRequestedAt > FERTILITY_TIMEOUT_MS) {
            if (onLinedHalf) {
              subState = SS_FOLLOWING;
              blankStartedAt = 0;
              resetPID();
            } else {
              if (nodesInColumn >= 9) { subState = SS_COLUMN_END; uTurnStep = 0; }
              else if (nodesInColumn >= UNLINED_ROWS) { subState = SS_FIND_LINE; blankStartedAt = millis(); }
              else { driveStartTicks = getTrackEncoder(); subState = SS_DEAD_RECKON; }
            }
          }
          break;
        }

        // ------ Drive encoder-counted distance to position planter over hole ------
        case SS_DRIVE_TO_HOLE: {
          long currentTicks = getTrackEncoder();
          long tickTarget   = encoderAtIR + ticksToHole;
          long encError     = tickTarget - currentTicks;

          if (encError > 5) {
            driveStraight((int)(600 * voltageScale));
          } else {
            stopTracks();
            planterTarget = getPlanterEncoder() + ticksToPlant;
            subState = SS_PLANTING;
          }
          break;
        }

        // ------ Rotate planter to drop one seed ------
        case SS_PLANTING: {
          long planterPos = getPlanterEncoder();
          if (abs(planterPos - planterTarget) > 5) {
            setPlanter((int)(500 * voltageScale));
          } else {
            setPlanter(0);
            delay(500);
            resetPID();

            if (onLinedHalf) {
              subState = SS_FOLLOWING;
              blankStartedAt = 0;
            } else {
              if (nodesInColumn >= 9) { subState = SS_COLUMN_END; uTurnStep = 0; }
              else if (nodesInColumn >= UNLINED_ROWS) { subState = SS_FIND_LINE; blankStartedAt = millis(); }
              else { driveStartTicks = getTrackEncoder(); subState = SS_DEAD_RECKON; }
            }
          }
          break;
        }

        // ------ U-turn at end of column to reach the next one ------
        // Step 0: turn 90°  Step 1: drive one node spacing
        // Step 2: turn 90°  Step 3: enter next column
        case SS_COLUMN_END: {
          switch (uTurnStep) {
            case 0: {
              float angle = (gridDir > 0) ? 90.0 : -90.0;
              startTurn(angle);
              turnActive = true;
              uTurnStep = 1;
              break;
            }
            case 1: {
              // First turn done — drive one node spacing sideways
              driveStartTicks = getTrackEncoder();
              driveStraight(baseSpeed);
              uTurnStep = 2;
              break;
            }
            case 2: {
              if (abs(getTrackEncoder() - driveStartTicks) >= NODE_SPACING_TICKS) {
                stopTracks();
                delay(200);
                float angle = (gridDir > 0) ? 90.0 : -90.0;
                startTurn(angle);
                turnActive = true;
                uTurnStep = 3;
              } else {
                driveStraight(baseSpeed);
              }
              break;
            }
            case 3: {
              // Second turn done — now facing along the next column
              gridDir = -gridDir;
              columnsDone++;
              nodesInColumn = 0;
              uTurnStep = 0;
              blankStartedAt = 0;
              resetPID();

              if (gridDir > 0) {
                // Going 9→1: start on lined rows
                onLinedHalf = true;
                subState = SS_FIND_LINE;
                blankStartedAt = millis();
              } else {
                // Going 1→9: start on unlined rows
                onLinedHalf = false;
                driveStartTicks = getTrackEncoder();
                subState = SS_DEAD_RECKON;
              }
              break;
            }
          }
          break;
        }

        // ------ Unlined half: drive fixed distance to next node ------
        case SS_DEAD_RECKON: {
          if (abs(getTrackEncoder() - driveStartTicks) >= NODE_SPACING_TICKS) {
            stopTracks();
            nodesInColumn++;
            irDetectedAt = millis();
            subState = SS_DR_ARRIVE;
          } else {
            driveStraight(baseSpeed);
          }
          break;
        }

        // ------ Unlined: at estimated node, scan for RFID ------
        case SS_DR_ARRIVE: {
          stopTracks();
          String uid = readRFIDTag();
          if (uid.length() > 0) {
            detectedUID = uid;
            encoderAtIR = getTrackEncoder();
            checkFertility(uid);
            fertilityRequestedAt = millis();
            irDetectedAt = 0;
            subState = SS_WAIT_FERTILITY;
            break;
          }

          // Scanning timeout (500ms) — no tag found, move on
          if (millis() - irDetectedAt > 500) {
            irDetectedAt = 0;
            if (nodesInColumn >= 9) {
              subState = SS_COLUMN_END;
              uTurnStep = 0;
            } else if (!onLinedHalf && nodesInColumn >= UNLINED_ROWS) {
              // Done with unlined rows, transition to line following
              subState = SS_FIND_LINE;
              blankStartedAt = millis();
            } else {
              driveStartTicks = getTrackEncoder();
              subState = SS_DEAD_RECKON;
            }
          }
          break;
        }

        default: break;
      }
      break;
    }

    // ============================================================
    // STAGE 4: ARENA RETURN
    // Turn 180°, line follow back to base-side edge, turn left 90°
    // toward Airlock B, drive forward scanning for re-entry RFID
    // tag, send openAirlock(uid, 'B'), enter tunnel.
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
          // Turn just completed
          resetPID();
          blankStartedAt = 0;
          returnStep = 2;
          break;
        }

        case 2: {
          // Line follow back toward base edge
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

          // Also scan for RFID while returning — might hit re-entry tag
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
          // Turn left 90° toward Airlock B (re-entry tunnel)
          startTurn(-90.0);
          turnActive = true;
          returnStep = 4;
          break;
        }

        case 4: {
          // Turn just completed → start seeking tunnel
          returnStep = 5;
          break;
        }

        case 5: {
          // Drive forward looking for Airlock B tunnel (TOF detects walls)

          // Scan for re-entry RFID tag and send airlock request
          if (!airlockBRequested) {
            String uid = readRFIDTag();
            if (uid.length() > 0) {
              openAirlock(uid, 'B');
              airlockBRequested = true;
            }
          }

          // Check front for wall (arena boundary)
          float frontDist = getFrontDistanceCm();
          if (frontDist > 0 && frontDist < 8.0) {
            stopTracks();
            break;
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
    // Wall follow through Airlock B tunnel back to base.
    // ============================================================
    case STAGE_TUNNEL_IN: {
      if (runWallFollow()) {
        clearEmergency();
        stage = STAGE_BASE_PARK;
        subState = SS_FIND_LINE;
        blankStartedAt = millis();
        resetPID();
      }
      break;
    }

    // ============================================================
    // STAGE 6: BASE PARK
    // Follow line from Airlock B entrance to parking area.
    // Scans RFID tags along the way (tag B for opening airlock
    // to help other robots re-enter).
    // ============================================================
    case STAGE_BASE_PARK: {
      switch (subState) {
        case SS_FIND_LINE: {
          driveStraight(baseSpeed);
          if (!isBlank()) {
            subState = SS_FOLLOWING;
            blankStartedAt = 0;
            resetPID();
          }
          if (millis() - blankStartedAt > 5000) {
            stopTracks();
            stage = STAGE_DONE;
          }
          break;
        }

        case SS_FOLLOWING: {
          runLineFollower();

          // Read RFID tags along parking path
          String uid = readRFIDTag();
          if (uid.length() > 0) {
            openAirlock(uid, 'B');
          }

          if (isJunction()) {
            stopTracks();
            subState = SS_JUNCTION;
          }

          // Detect parking area (line ends)
          if (isBlank()) {
            if (blankStartedAt == 0) blankStartedAt = millis();
            if (millis() - blankStartedAt > 2000) {
              stopTracks();
              stage = STAGE_DONE;
            }
          } else {
            blankStartedAt = 0;
          }
          break;
        }

        case SS_JUNCTION: {
          angleRight((int)(500 * voltageScale));
          delay(200);
          resetPID();
          blankStartedAt = 0;
          subState = SS_FOLLOWING;
          break;
        }

        default: break;
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
