// ============================================================
// control_test.ino
// Unified test sketch for all 8 competition control tasks.
//
// Kill switch (pin 39) toggles start/stop.
//   - Press while STOPPED: start current mode (first time)
//     or advance to next mode and start (subsequent).
//   - Press while RUNNING: stop the robot.
//
// Serial Monitor shows current mode and status.
// ============================================================

#include <Wire.h>
#include <MFRC522_I2C.h>
#include "motors.h"
#include "sensors.h"
#include "wifi_utils.h"

// ===== RFID =====
MFRC522_I2C mfrc522(0x28, -1, &Wire1);

// ===== Pins =====
#define KILL_BUTTON_PIN   39
#define LED_RED_PIN       38
#define LED_GREEN_PIN     40

// ===== Modes =====
enum Mode {
  MODE_LINE_TRACKING,       // Task 1
  MODE_INTERSECTION_TAG,    // Task 2
  MODE_GRID_NAV,            // Task 3
  MODE_DEAD_RECKONING,      // Task 4
  MODE_RAMP,                // Task 5
  MODE_WALL_FOLLOWING,      // Task 6
  MODE_OBSTACLE_AVOIDANCE,  // Task 7
  MODE_REVIVAL,             // Task 8
  MODE_COUNT
};

static const char* modeNames[] = {
  "1: Line Tracking",
  "2: Intersection & Tag",
  "3: Grid Navigation",
  "4: Dead Reckoning",
  "5: Ramp Control",
  "6: Wall Following",
  "7: Obstacle Avoidance",
  "8: Revival"
};

Mode currentMode = MODE_LINE_TRACKING;

// ===== Kill Switch State =====
bool isKilledLocal = true;
int  killBtnState  = HIGH;
int  lastBtnState  = HIGH;
unsigned long lastDebounceTime = 0;
const unsigned long debounceDelay = 50;
bool firstRun = true;

// ===== LED =====
unsigned long ledPrevMillis = 0;
const long blinkInterval = 500;
bool redLedOn = false;

// ===== Line Following PID =====
float Kp_line = 1.0, Ki_line = 0.0, Kd_line = 0.0;
const int baseSpeed = (int)(500 * 6 / 7.2);
const int maxSpeed  = (int)(800 * 6 / 7.2);
const int minSpeed  = -(int)(800 * 6 / 7.2);
const int setpoint  = 5500;

float integral_line = 0, prevError_line = 0;
unsigned long prevTime_line = 0;

// ===== Junction Detection =====
long lastJunctionTick = -9999;
const long junctionCooldownTicks = 300;

// ===== Encoder-Based Driving =====
long driveStartTicks  = 0;
long driveTicksTarget = 0;
bool driveActive = false;
const float TICKS_PER_MM = 1400.0 / (38.5 * PI);

// ===== Turn Management =====
bool turnActive = false;

// ===== Task 2: Intersection & Tag =====
int t2Junctions = 0;
int t2State = 0;  // 0=following, 1=junction handling

// ===== Task 3: Grid Navigation =====
int  gridJunctions = 0;
bool gridTurning   = false;
bool gridDone      = false;

// ===== Task 4: Dead Reckoning =====
int  drSegment    = 0;   // 0=first 2 fwd, 1=1 fwd, 2=last 2 fwd
int  drNodesInSeg = 0;
bool drDone       = false;

// ===== Task 5: Ramp =====
bool rampDone = false;
unsigned long wallLostAt = 0;

// ===== Task 6: Wall Following PID =====
float Kp_wall = 2.5, Ki_wall = 0.0, Kd_wall = 2.0;
const int WALL_TARGET_MM  = 100;
const int WALL_BASE_PWM   = 550;
const int WALL_DEADBAND   = 60;
float integral_wall = 0, prevError_wall = 0;
unsigned long lastWallControl = 0;
float smoothedRight = 100.0, smoothedLeft = 100.0;

// ===== Task 7: Obstacle Avoidance =====
int  oaStep          = 0;
bool oaStepStarted   = false;
int  oaResumeJunctions = 0;

// ===== Task 8: Revival =====
bool revivalDone = false;

// ================================================================
// UTILITY FUNCTIONS
// ================================================================

float computeLinePID(float position) {
  unsigned long now = millis();
  float dt = (now - prevTime_line) / 1000.0;
  if (dt <= 0) return 0;
  prevTime_line = now;
  float error = setpoint - position;
  integral_line += error * dt;
  float derivative = (error - prevError_line) / dt;
  prevError_line = error;
  return (Kp_line * error) + (Ki_line * integral_line) + (Kd_line * derivative);
}

void resetLinePID() {
  integral_line  = 0;
  prevError_line = 0;
  prevTime_line  = millis();
}

void runLineFollower() {
  uint16_t position = readIRPosition();
  float correction  = computeLinePID(position);
  int leftSpeed  = constrain(baseSpeed - correction, minSpeed, maxSpeed);
  int rightSpeed = constrain(baseSpeed + correction, minSpeed, maxSpeed);
  setLeftTrack(leftSpeed);
  setRightTrack(rightSpeed);
}

bool isJunction() {
  readIRPosition();
  uint8_t lastIdx = getIRSensorCount() - 1;
  return (getIRValue(0) > 800 && getIRValue(lastIdx) > 800);
}

bool isNewJunction() {
  if (!isJunction()) return false;
  long tick = getTrackEncoder();
  if (abs(tick - lastJunctionTick) < junctionCooldownTicks) return false;
  lastJunctionTick = tick;
  return true;
}

void driveStraight(int speed) {
  setLeftTrack(speed);
  setRightTrack(speed);
}

void startDriveDist(long mm) {
  driveStartTicks  = getTrackEncoder();
  driveTicksTarget = (long)(mm * TICKS_PER_MM);
  driveActive = true;
  driveStraight(baseSpeed);
}

bool updateDriveDist() {
  if (!driveActive) return true;
  long pos = getTrackEncoder();
  if (abs(pos - driveStartTicks) >= driveTicksTarget) {
    stopTracks();
    driveActive = false;
    return true;
  }
  return false;
}

void beginTurnDeg(float degrees) {
  startTurn(degrees);
  turnActive = true;
}

bool checkTurnDone() {
  if (!turnActive) return true;
  if (updateTurn()) {
    turnActive = false;
    return true;
  }
  return false;
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
// MODE RESET
// ================================================================

void resetModeState() {
  stopTracks();
  resetLinePID();
  turnActive  = false;
  driveActive = false;
  lastJunctionTick = -9999;

  t2Junctions = 0;
  t2State = 0;

  gridJunctions = 0;
  gridTurning   = false;
  gridDone      = false;

  drSegment    = 0;
  drNodesInSeg = 0;
  drDone       = false;

  rampDone   = false;
  wallLostAt = 0;

  integral_wall   = 0;
  prevError_wall  = 0;
  lastWallControl = millis();
  smoothedRight   = WALL_TARGET_MM;
  smoothedLeft    = WALL_TARGET_MM;

  oaStep           = 0;
  oaStepStarted    = false;
  oaResumeJunctions = 0;

  revivalDone = false;
}

// ================================================================
// TASK 1: Standard Line Tracking
// ================================================================

void runLineTracking() {
  runLineFollower();
}

// ================================================================
// TASK 2: Intersection & Tag Alignment (base exit)
// Follow line in base, turn right at junction 1, right again at
// junction 2, read RFID to request airlock opening, continue.
// ================================================================

void runIntersectionTag() {
  switch (t2State) {
    case 0: {  // FOLLOWING
      runLineFollower();

      if (mfrc522.PICC_IsNewCardPresent() && mfrc522.PICC_ReadCardSerial()) {
        String uid = "";
        for (byte i = 0; i < mfrc522.uid.size; i++) {
          if (mfrc522.uid.uidByte[i] < 0x10) uid += "0";
          uid += String(mfrc522.uid.uidByte[i], HEX);
        }
        uid.toUpperCase();
        mfrc522.PICC_HaltA();
        mfrc522.PCD_StopCrypto1();
        stopTracks();
        openAirlock(uid, 'A');
      }

      if (isJunction()) {
        stopTracks();
        t2State = 1;
        t2Junctions++;
        Serial.print("[T2] Junction ");
        Serial.println(t2Junctions);
      }
      break;
    }
    case 1: {  // JUNCTION — handle on next loop after stop
      setLeftTrack(500);
      setRightTrack(0);
      t2State = 0;
      digitalWrite(LED_GREEN_PIN, LOW);
      digitalWrite(LED_RED_PIN, LOW);
      delay(100);
      break;
    }
  }
}

// ================================================================
// TASK 3: Solid Grid Navigation
// Forward 2 nodes → right 90° → forward 1 node → left 90° →
// forward 2 nodes → stop.  Nodes counted by RFID tags.
// ================================================================

void runGridNav() {
  if (gridDone) return;

  if (gridTurning) {
    if (checkTurnDone()) {
      gridTurning = false;
      resetLinePID();
      // Serial.println("[T3] Turn complete");
    }
    return;
  }

  runLineFollower();

  String uid = readRFIDTag();
  if (uid.length() > 0) {
    gridJunctions++;
    // Serial.print("[T3] RFID node ");
    // Serial.println(gridJunctions);

    if (gridJunctions == 2) {
      stopTracks();
      driveStraight((int)(600 * 6 / 7.2));
      delay(200);
      stopTracks();
      beginTurnDeg(90.0);
      gridTurning = true;
    } else if (gridJunctions == 3) {
      stopTracks();
      driveStraight((int)(600 * 6 / 7.2));
      delay(200);
      stopTracks();
      beginTurnDeg(-90.0);
      gridTurning = true;
    } else if (gridJunctions == 5) {
      stopTracks();
      gridDone = true;
      // Serial.println("[T3] Complete!");
    }
  }
}

// ================================================================
// TASK 4: Open-Field Dead Reckoning
// Same manoeuvre as Task 3 but no lines — drive straight and
// count RFID tags at each node to know when to turn.
// Segments: fwd 2 nodes → R90 → fwd 1 node → L90 → fwd 2 nodes.
// ================================================================

void runDeadReckoning() {
  if (drDone) return;

  if (turnActive) {
    if (checkTurnDone()) {
      drNodesInSeg = 0;
      driveStraight(baseSpeed);
      Serial.println("[T4] Turn complete, driving");
    }
    return;
  }

  driveStraight(baseSpeed);

  String uid = readRFIDTag();
  if (uid.length() > 0) {
    drNodesInSeg++;
    // Serial.print("[T4] RFID node (seg ");
    // Serial.print(drSegment);
    // Serial.print(" count ");
    // Serial.print(drNodesInSeg);
    // Serial.println(")");

    if (drSegment == 0 && drNodesInSeg >= 2) {
      stopTracks();
      beginTurnDeg(90.0);
      drSegment = 1;
      // Serial.println("[T4] Turn R 90");
    } else if (drSegment == 1 && drNodesInSeg >= 1) {
      stopTracks();
      beginTurnDeg(-90.0);
      drSegment = 2;
      // Serial.println("[T4] Turn L 90");
    } else if (drSegment == 2 && drNodesInSeg >= 2) {
      stopTracks();
      drDone = true;
      // Serial.println("[T4] Complete!");
    }
  }
}

// ================================================================
// TASK 5: Ramp Incline/Decline Control
// Drive through the airlock tunnel, centering between walls using
// TOF sensors.  Stop when both walls disappear (entered arena).
// ================================================================

void runRamp() {
  if (rampDone) return;

  readTOFSensors();
  unsigned long dL = getTOFLeftDist();
  unsigned long dR = getTOFRightDist();
  bool seeL = isTOFLeftFresh() && dL > 0 && dL < 500;
  bool seeR = isTOFRightFresh() && dR > 0 && dR < 500;

  if (!seeL && !seeR) {
    if (wallLostAt == 0) wallLostAt = millis();
    if (millis() - wallLostAt > 500) {
      stopTracks();
      rampDone = true;
      // Serial.println("[T5] Exited airlock");
      return;
    }
    driveStraight(baseSpeed);
  } else {
    wallLostAt = 0;
    int speed = (int)(500 * 6 / 7.2);
    if (seeL && seeR) {
      int error = (int)dL - (int)dR;
      int correction = constrain(error / 2, -150, 150);
      setLeftTrack(constrain(speed + correction, 100, maxSpeed));
      setRightTrack(constrain(speed - correction, 100, maxSpeed));
    } else {
      driveStraight(speed);
    }
  }
}

// ================================================================
// TASK 6: Wall Following
// PID wall following using TOF side sensors. Ported from
// wallfollowing_2.ino with front-obstacle stop.
// ================================================================

void runWallFollowing() {
  readTOFSensors();

  unsigned long currentTime = millis();
  float deltaTime = (currentTime - lastWallControl) / 1000.0;
  if (deltaTime < 0.02) return;

  float frontDist = getFrontDistanceCm();
  if (frontDist > 0 && frontDist <= 10.0) {
    stopTracks();
    integral_wall  = 0;
    prevError_wall = 0;
    lastWallControl = currentTime;
    return;
  }

  unsigned long dR = getTOFRightDist();
  unsigned long dL = getTOFLeftDist();
  bool seeR = isTOFRightFresh() && dR > 0 && dR < 500;
  bool seeL = isTOFLeftFresh()  && dL > 0 && dL < 500;

  if (seeR) smoothedRight = 0.3f * (float)dR + 0.7f * smoothedRight;
  if (seeL) smoothedLeft  = 0.3f * (float)dL + 0.7f * smoothedLeft;

  float error = 0.0;
  bool followingRight = true;

  if (seeR && seeL) {
    followingRight = (smoothedRight <= smoothedLeft);
    error = (followingRight ? smoothedRight : smoothedLeft) - WALL_TARGET_MM;
  } else if (seeR) {
    error = smoothedRight - WALL_TARGET_MM;
  } else if (seeL) {
    followingRight = false;
    error = smoothedLeft - WALL_TARGET_MM;
  } else {
    driveStraight(WALL_BASE_PWM);
    integral_wall   = 0;
    lastWallControl = currentTime;
    return;
  }

  integral_wall += error * deltaTime;
  integral_wall = constrain(integral_wall, -300.0f, 300.0f);
  float derivative = (error - prevError_wall) / deltaTime;
  float correction = (Kp_wall * error) + (Ki_wall * integral_wall) + (Kd_wall * derivative);

  // Angled-approach gate: closing on wall at an angle, don't steer further in
  if (error > 0.0f && derivative < 0.0f) {
    correction = min(correction, 0.0f);
  }

  if (abs(error) > 5.0f) {
    if (correction > 0) correction += WALL_DEADBAND;
    if (correction < 0) correction -= WALL_DEADBAND;
  }

  int cmdL = WALL_BASE_PWM, cmdR = WALL_BASE_PWM;
  if (followingRight) {
    cmdL += (int)correction;
    cmdR -= (int)correction;
  } else {
    cmdL -= (int)correction;
    cmdR += (int)correction;
  }

  const int MIN_FWD = 100;
  cmdL = constrain(cmdL, MIN_FWD, maxSpeed);
  cmdR = constrain(cmdR, MIN_FWD, maxSpeed);

  setLeftTrack(cmdL);
  setRightTrack(cmdR);

  prevError_wall  = error;
  lastWallControl = currentTime;
}

// ================================================================
// TASK 7: Obstacle Detection & Avoidance
// Line follow until front ultrasonic detects obstacle, execute a
// rectangular swerve (left), then resume line follow for 3 nodes.
// ================================================================

void runObstacleAvoidance() {
  const long NODE_MM = 250;

  switch (oaStep) {
    case 0: {
      runLineFollower();
      float dist = getFrontDistanceCm();
      if (dist > 0 && dist < 20.0) {
        stopTracks();
        // Serial.println("[T7] Obstacle detected, swerving left");
        oaStep = 1;
        oaStepStarted = false;
      }
      break;
    }
    case 1: // turn left 90
      if (!oaStepStarted) { beginTurnDeg(-90.0); oaStepStarted = true; }
      if (checkTurnDone()) { oaStep = 2; oaStepStarted = false; }
      break;
    case 2: // sidestep one lane (250 mm)
      if (!oaStepStarted) { startDriveDist(NODE_MM); oaStepStarted = true; }
      if (updateDriveDist()) { oaStep = 3; oaStepStarted = false; }
      break;
    case 3: // turn right 90 (face original direction)
      if (!oaStepStarted) { beginTurnDeg(90.0); oaStepStarted = true; }
      if (checkTurnDone()) { oaStep = 4; oaStepStarted = false; }
      break;
    case 4: // drive past obstacle (500 mm)
      if (!oaStepStarted) { startDriveDist(2 * NODE_MM); oaStepStarted = true; }
      if (updateDriveDist()) { oaStep = 5; oaStepStarted = false; }
      break;
    case 5: // turn right 90 (face back toward original lane)
      if (!oaStepStarted) { beginTurnDeg(90.0); oaStepStarted = true; }
      if (checkTurnDone()) { oaStep = 6; oaStepStarted = false; }
      break;
    case 6: // return to original lane (250 mm)
      if (!oaStepStarted) { startDriveDist(NODE_MM); oaStepStarted = true; }
      if (updateDriveDist()) { oaStep = 7; oaStepStarted = false; }
      break;
    case 7: // turn left 90 (face original direction again)
      if (!oaStepStarted) { beginTurnDeg(-90.0); oaStepStarted = true; }
      if (checkTurnDone()) {
        oaStep = 8;
        oaStepStarted = false;
        oaResumeJunctions = 0;
        resetLinePID();
        // Serial.println("[T7] Swerve complete, resuming line follow");
      }
      break;
    case 8: {
      runLineFollower();
      if (isNewJunction()) {
        oaResumeJunctions++;
        // Serial.print("[T7] Post-swerve junction ");
        // Serial.println(oaResumeJunctions);
        if (oaResumeJunctions >= 3) {
          stopTracks();
          oaStep = 9;
          // Serial.println("[T7] Complete!");
        }
      }
      break;
    }
    case 9:
      break;
  }
}

// ================================================================
// TASK 8: Touch-Based Robot Revival
// Line follow toward target, decelerate proportionally on
// approach, stop on contact (ultrasonic < 3 cm).
// ================================================================

void runRevival() {
  if (revivalDone) return;

  float dist = getFrontDistanceCm();

  if (dist > 0 && dist < 3.0) {
    stopTracks();
    revivalDone = true;
    // Serial.println("[T8] Contact made!");
    return;
  }

  if (dist > 0 && dist < 30.0) {
    int speed = map((long)(dist * 10), 30, 300, 150, baseSpeed);
    speed = constrain(speed, 150, baseSpeed);
    
    driveStraight(speed);
  } else {
    driveStraight(500);
  }
}

// ================================================================
// KILL SWITCH & LED
// ================================================================

void checkKillButton() {
  int reading = digitalRead(KILL_BUTTON_PIN);
  if (reading != lastBtnState) lastDebounceTime = millis();
  lastBtnState = reading;

  if ((millis() - lastDebounceTime) > debounceDelay && reading != killBtnState) {
    killBtnState = reading;
    if (killBtnState == LOW) {
      if (!isKilledLocal) {
        // Running → stop
        isKilledLocal = true;
        stopTracks();
        // Serial.print("[Kill] STOPPED | Mode: ");
        // Serial.println(modeNames[currentMode]);
      } else {
        // Stopped → advance mode (skip on first press) and start
        if (!firstRun) {
          currentMode = (Mode)((currentMode + 1) % MODE_COUNT);
        }
        firstRun = false;
        resetModeState();
        isKilledLocal = false;
        // Serial.print("[Start] RUNNING | Mode: ");
        // Serial.println(modeNames[currentMode]);
      }
    }
  }
}

void updateLED() {
  bool killed = isKilledLocal;
  if (!killed) {
    digitalWrite(LED_RED_PIN, LOW);
    digitalWrite(LED_GREEN_PIN, HIGH);
    redLedOn = false;
    return;
  }
  unsigned long now = millis();
  if (now - ledPrevMillis >= (unsigned long)blinkInterval) {
    ledPrevMillis = now;
    redLedOn = !redLedOn;
    digitalWrite(LED_RED_PIN,   redLedOn ? HIGH : LOW);
    digitalWrite(LED_GREEN_PIN, LOW);
  }
}

// ================================================================
// SETUP & LOOP
// ================================================================

void setup() {
  Serial.begin(115200);
  delay(2000);

  Wire1.begin();
  mfrc522.PCD_Init();

  delay(500);
  initMotors();

  initSensors();
  // Serial.println("[IR] Calibrating - sweep sensors across line...");
  initIRArray();
  // Serial.println("[IR] Calibration done.");

  pinMode(LED_RED_PIN,     OUTPUT);
  pinMode(LED_GREEN_PIN,   OUTPUT);
  pinMode(KILL_BUTTON_PIN, INPUT_PULLUP);
  digitalWrite(LED_RED_PIN,   LOW);
  digitalWrite(LED_GREEN_PIN, LOW);

  initWifi();
  setupGrid();
  register_bot();

  // Serial.println("=== CONTROL TEST READY ===");
  // Serial.print("Current mode: ");
  // Serial.println(modeNames[currentMode]);
  // Serial.println("Press KILL SWITCH to start. Each subsequent stop+press cycles to next mode.");
  // Serial.println("Modes: 1)LineTrack 2)Intersection 3)GridNav 4)DeadReck 5)Ramp 6)WallFollow 7)Obstacle 8)Revival");
}

void loop() {
  checkKillButton();
  updateLED();

  if (currentMode == MODE_INTERSECTION_TAG) loopWifi();

  bool killed = isKilledLocal;
  static bool wasKilled = false;
  if (killed && !wasKilled) {
    stopTracks();
    stopPlanter();
    wasKilled = true;
  } else if (!killed) {
    wasKilled = false;
  }

  if (killed) return;

  switch (currentMode) {
    case MODE_LINE_TRACKING:      runLineTracking(); break;
    case MODE_INTERSECTION_TAG:   runIntersectionTag(); break;
    case MODE_GRID_NAV:           runGridNav(); break;
    case MODE_DEAD_RECKONING:     runDeadReckoning(); break;
    case MODE_RAMP:               runRamp(); break;
    case MODE_WALL_FOLLOWING:     runWallFollowing(); break;
    case MODE_OBSTACLE_AVOIDANCE: runObstacleAvoidance(); break;
    case MODE_REVIVAL:            runRevival(); break;
    default: break;
  }
}
