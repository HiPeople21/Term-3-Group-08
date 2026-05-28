// =============================================================================
// UCL RAI Robotics Challenge 2026 — Main Firmware
// Platform  : Arduino GIGA R1 WiFi + Motoron M3S550
//
// Control checklist → stage/state mapping:
//  Task 1  Standard Line Tracking            BASE        → FOLLOWING
//  Task 2  Intersection & Tag Alignment      BASE        → JUNCTION_HANDLING → AWAITING_EXIT_CLEARANCE
//  Task 3  Solid Grid Navigation             LINED       → FOLLOWING
//  Task 4  Open-Field Dead Reckoning         BLANK       → DEAD_RECKONING
//  Task 5  Ramped Incline/Decline            TUNNEL      → ASCENDING / DESCENDING
//  Task 6  Wall Following                    WALL_FOLLOW → WALL_FOLLOWING
//  Task 7  Obstacle Detection & Avoidance    LINED/BLANK → OBSTACLE_DETECTED → OBSTACLE_AVOIDING
//  Task 8  Touch-Based Robot Revival         REVIVAL     → REVIVAL_APPROACH → REVIVAL_CONTACT
// =============================================================================

#include <Wire.h>
#include <MFRC522_I2C.h>
#include "motors.h"
#include "sensors.h"
#include "wifi_utils.h"

// --- RFID (Wire1, I2C address 0x28) ---
MFRC522_I2C mfrc522(0x28, -1, &Wire1);

// --- Pin assignments ---
#define KILL_BUTTON_PIN    39
#define REVIVE_BUTTON_PIN  48
#define LED_RED_PIN        38
#define LED_GREEN_PIN      40

// --- Kill switch state ---
static bool          isKilledLocal     = false;
static int           killBtnState      = HIGH;
static int           lastBtnState      = HIGH;
static unsigned long lastDebounceTime  = 0;
static const unsigned long DEBOUNCE_MS = 50;

// --- LED blink state ---
static unsigned long previousMillis    = 0;
static const long    BLINK_INTERVAL_MS = 500;
static bool          redLedOn          = false;

// --- Revival button ---
static int           reviveBtnState    = HIGH;
static int           lastReviveState   = HIGH;
static unsigned long lastReviveDebounce = 0;

// --- Manual jog speed ---
static const int TRACK_SPEED = 800;

// --- PID gains (line following) ---
float Kp = 1.0f;
float Ki = 0.0f;
float Kd = 0.0f;

// --- Wall-following PID gains (Task 6) ---
float Kp_wall = 1.0f;
float Ki_wall = 0.0f;
float Kd_wall = 0.0f;

// --- Speed limits ---
const int BASE_SPEED    = 800 * 6 / 7.2;
const int SLOW_SPEED    = 400 * 6 / 7.2;
const int REVIVAL_SPEED = 200 * 6 / 7.2;   // Task 8: decelerated approach
const int MAX_SPEED     = 800 * 6 / 7.2;
const int MIN_SPEED     = -(800 * 6 / 7.2);

// --- IR sensor ---
const int  IR_SETPOINT  = 5500;             // centred on 9-sensor array (0–8000)

// --- Planting geometry (empirical: ~91 ticks/cm) ---
const long TICKS_TO_HOLE    = 1355;         // encoder ticks from IR trigger to hole centre
const long TICKS_TO_PLANT   = 233;          // planter ticks for one seed drop

// --- Dead-reckoning grid (Task 4: ~25 cm per square) ---
const long TICKS_PER_SQUARE = 2275;

// --- Timing windows ---
const unsigned long IR_WINDOW_MS               = 1000;
const unsigned long FERTILITY_TIMEOUT_MS       = 5000;
const unsigned long EXIT_CLEARANCE_TIMEOUT_MS  = 10000; // Task 2: max wait for airlock open

// --- Wall following (Task 6) ---
const int WALL_TARGET_MM = 150;             // desired offset from wall in mm

// --- Obstacle detection (Task 7) ---
const int OBSTACLE_THRESHOLD_MM = 200;      // front ultrasonic trigger distance

// --- Revival approach (Task 8) ---
const long REVIVAL_APPROACH_TICKS = 5000;   // TODO: measure to stranded robot position

// --- Junction cooldown ---
const long JUNCTION_COOLDOWN_TICKS = 200;

// -----------------------------------------------------------------------

enum Stage {
  BASE,         // Tasks 1–2  : base room — line follow → junction → RFID B-tag → exit clearance
  LINED,        // Task  3    : left arena — solid grid line following + RFID planting
  BLANK,        // Task  4    : right arena — dead-reckoning between nodes + RFID planting
  TUNNEL,       // Task  5    : airlock ramp — ascend / descend
  WALL_FOLLOW,  // Task  6    : arena/airlock perimeter wall following
  REVIVAL,      // Task  8    : approach & contact stranded robot
  RETURNING     // Return to base
};

enum State {
  // --- navigation (shared) ---
  FOLLOWING,                // PID line following
  JUNCTION_HANDLING,        // clear a detected junction

  // --- base room (Task 2) ---
  AWAITING_EXIT_CLEARANCE,  // stopped over RFID B-tag; waiting for server to open airlock

  // --- RFID / planting (LINED & BLANK) ---
  WAITING_FOR_RFID,         // hole detected, driving toward RFID tag window
  WAITING_FOR_FERTILITY,    // RFID read; awaiting server fertility callback
  DRIVING_TO_HOLE,          // fertile confirmed; driving encoder-distance to hole centre
  PLANTING,                 // rotating planter motor by TICKS_TO_PLANT

  // --- dead reckoning (BLANK, Task 4) ---
  DEAD_RECKONING,           // encoder-tick navigation between nodes (no line)

  // --- obstacle avoidance (LINED / BLANK, Task 7) ---
  OBSTACLE_DETECTED,        // front sensor below threshold — plan avoidance
  OBSTACLE_AVOIDING,        // executing avoidance manoeuvre

  // --- tunnel (TUNNEL, Task 5) ---
  ASCENDING,                // climbing ramp at reduced speed
  DESCENDING,               // descending ramp at reduced speed

  // --- wall following (WALL_FOLLOW, Task 6) ---
  WALL_FOLLOWING,           // ToF/ultrasonic PID to maintain WALL_TARGET_MM offset

  // --- revival (REVIVAL, Task 8) ---
  REVIVAL_APPROACH,         // slow deceleration approach toward stranded robot
  REVIVAL_CONTACT,          // contact made — trigger revival signal
};

Stage stage = BASE;
State state = FOLLOWING;

// -----------------------------------------------------------------------

// --- PID state ---
float integral   = 0;
float prevError  = 0;
unsigned long prevTime = 0;

// --- Wall-follow PID state ---
float wallIntegral  = 0;
float wallPrevError = 0;

bool running = false;

int  prevMiddleValue = 0;

long encoderAtIR           = 0;
long planterTarget         = 0;
long deadReckonTarget      = 0;   // encoder target for current dead-reckoning move
long revivalApproachTarget = 0;
long lastJunctionTick      = -9999;

unsigned long irDetectedAt              = 0;
unsigned long fertilityRequestedAt      = 0;
unsigned long exitClearanceRequestedAt  = 0;

String detectedUID = "";

// -----------------------------------------------------------------------

void revive() {
  // TODO: implement revival signal (flash LED, send UDP, trigger buzzer, etc.)
  Serial.println("[Revival] Revived!");
}

void checkReviveButton() {
  int reading = digitalRead(REVIVE_BUTTON_PIN);
  if (reading != lastReviveState) lastReviveDebounce = millis();
  lastReviveState = reading;

  if ((millis() - lastReviveDebounce) > DEBOUNCE_MS && reading != reviveBtnState) {
    reviveBtnState = reading;
    if (reviveBtnState == LOW) {
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

  unsigned long now = millis();
  if (now - previousMillis >= (unsigned long)BLINK_INTERVAL_MS) {
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

  if ((millis() - lastDebounceTime) > DEBOUNCE_MS && reading != killBtnState) {
    killBtnState = reading;
    if (killBtnState == LOW) {
      isKilledLocal = !isKilledLocal;
      Serial.print("[Kill Btn] ");
      Serial.println(isKilledLocal ? "KILLED" : "ENABLED");
      if (isKilledLocal) { stopTracks(); stopPlanter(); }
    }
  }
}

// --- Line following helpers ---

float computePID(float position) {
  unsigned long now = millis();
  float dt = (now - prevTime) / 1000.0f;
  if (dt <= 0) return 0;
  prevTime = now;

  float error      = IR_SETPOINT - position;
  integral        += error * dt;
  float derivative = (error - prevError) / dt;
  prevError        = error;

  return (Kp * error) + (Ki * integral) + (Kd * derivative);
}

void resetPID() {
  integral  = 0;
  prevError = 0;
  prevTime  = millis();
}

void runLineFollower() {
  uint16_t position  = readIRPosition();
  float    correction = computePID(position);

  int leftSpeed  = constrain((int)(BASE_SPEED - correction), MIN_SPEED, MAX_SPEED);
  int rightSpeed = constrain((int)(BASE_SPEED + correction), MIN_SPEED, MAX_SPEED);

  setLeftTrack(leftSpeed);
  setRightTrack(rightSpeed);
}

bool checkForHole() {
  uint8_t midIdx      = getIRSensorCount() / 2;
  int     middleValue = getIRValue(midIdx);
  bool    inHole      = (middleValue >= 100 && middleValue <= 400);
  bool    wasOutside  = (prevMiddleValue < 100 || prevMiddleValue > 400);
  prevMiddleValue     = middleValue;
  return inHole && wasOutside;
}

bool isJunction() {
  readIRPosition();
  uint8_t lastIdx = getIRSensorCount() - 1;
  return (getIRValue(0) > 800 && getIRValue(lastIdx) > 800);
}

bool obstacleAhead() {
  // TODO: replace with real sensor call once getUltrasonicDistanceMM() is
  //       added to sensors.h / sensors.cpp
  // return getUltrasonicDistanceMM() < OBSTACLE_THRESHOLD_MM;
  return false;
}

void driveStraight(int speed) {
  setLeftTrack(speed);
  setRightTrack(speed);
}

// -----------------------------------------------------------------------

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000);

  Wire1.begin();

  mfrc522.PCD_Init();
  Serial.println("[RFID] Reader ready.");

  initMotors();
  Serial.println("[Motors] Ready.");

  initSensors();
  Serial.println("[Sensors] TOF + Ultrasonic ready.");

  initIRArray();
  Serial.println("[IR] Ready.");

  pinMode(LED_RED_PIN,       OUTPUT);
  pinMode(LED_GREEN_PIN,     OUTPUT);
  pinMode(KILL_BUTTON_PIN,   INPUT_PULLUP);
  pinMode(REVIVE_BUTTON_PIN, INPUT_PULLUP);
  digitalWrite(LED_RED_PIN,   LOW);
  digitalWrite(LED_GREEN_PIN, HIGH);
  Serial.println("[Kill Switch] Hardware ready.");

  initWifi();

  setupGrid();
  register_bot();

  setFertilityCallback([](bool fertile) {
    if (state == WAITING_FOR_FERTILITY) {
      state = fertile ? DRIVING_TO_HOLE : FOLLOWING;
      Serial.print("[Fertility] ");
      Serial.println(fertile ? "FERTILE — driving to hole" : "NOT FERTILE — resuming");
    }
  });

  Serial.println("=== SYSTEM ONLINE ===");
  Serial.println("Serial: G=start  X=stop  W/S/A/D=jog  1=airlockA  2=airlockB");
}

// -----------------------------------------------------------------------

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

  if (!killed && Serial.available() > 0) {
    char cmd = Serial.read();
    if (cmd == 'g' || cmd == 'G') {
      running = true;
      resetPID();
      state = FOLLOWING;
      Serial.println("[CMD] Autonomy started.");
    } else if (cmd == 'x' || cmd == 'X') {
      running = false;
      stopTracks();
      Serial.println("[CMD] Stopped.");
    } else if (!running) {
      switch (cmd) {
        case 'w': case 'W': setLeftTrack(TRACK_SPEED);  setRightTrack(TRACK_SPEED);  break;
        case 's': case 'S': setLeftTrack(-TRACK_SPEED); setRightTrack(-TRACK_SPEED); break;
        case 'a': case 'A': setLeftTrack(-TRACK_SPEED); setRightTrack(TRACK_SPEED);  break;
        case 'd': case 'D': setLeftTrack(TRACK_SPEED);  setRightTrack(-TRACK_SPEED); break;
        case '1': openAirlockA(); break;
        case '2': openAirlockB(); break;
        case '3': { int i = Serial.parseInt(); seedPlanted(UIDs[i]); break; }
        case '4': { int i = Serial.parseInt(); checkFertility(UIDs[i]); break; }
      }
    }
  }

  if (running && !killed) {
    switch (stage) {

      // =====================================================================
      // BASE — Tasks 1 & 2
      // Line follow through base room → detect junction → align over RFID
      // B-tag → request exit clearance → enter arena.
      // =====================================================================
      case BASE:
        switch (state) {

          case FOLLOWING: {
            // Task 1: follow line from start toward exit door
            runLineFollower();

            if (isJunction() && (getTrackEncoder() - lastJunctionTick) > JUNCTION_COOLDOWN_TICKS) {
              stopTracks();
              state = JUNCTION_HANDLING;
              Serial.println("[Base] Junction detected.");
            }
            break;
          }

          case JUNCTION_HANDLING: {
            // Task 2: bifurcation in base — select correct branch.
            // TODO: implement branch-selection logic (e.g. use side IR arrays
            //       to detect which branch has a line, or always take left).
            driveStraight(SLOW_SPEED);
            delay(200);
            stopTracks();
            lastJunctionTick = getTrackEncoder();
            resetPID();
            irDetectedAt = millis();
            state = WAITING_FOR_RFID;
            Serial.println("[Base] Junction cleared — watching for RFID B-tag.");
            break;
          }

          case WAITING_FOR_RFID: {
            // Task 2: slow-follow until RFID B-tag detected
            runLineFollower();

            if (millis() - irDetectedAt > IR_WINDOW_MS) {
              state = FOLLOWING;
              Serial.println("[Base] RFID timeout — resuming.");
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

              // TODO: call correct WiFi helper to request exit clearance,
              //       e.g. openAirlockA() or a dedicated requestExit(detectedUID)
              exitClearanceRequestedAt = millis();
              state = AWAITING_EXIT_CLEARANCE;
              Serial.print("[Base] RFID B-tag: "); Serial.println(detectedUID);
            }
            break;
          }

          case AWAITING_EXIT_CLEARANCE: {
            // Task 2: stopped over B-tag; wait for server to confirm airlock open.
            // TODO: replace timeout fallback with a real server callback /
            //       isSystemEnabled() flag that signals the airlock is open.
            if (millis() - exitClearanceRequestedAt < EXIT_CLEARANCE_TIMEOUT_MS) {
              Serial.println("[Base] Awaiting exit clearance...");
              break;
            }
            Serial.println("[Base] Exit clearance received — entering arena.");
            // TODO: decide whether to go to TUNNEL (ramp first) or straight to LINED
            stage = LINED;
            state = FOLLOWING;
            resetPID();
            break;
          }

          default:
            state = FOLLOWING;
            break;
        }
        break;

      // =====================================================================
      // LINED — Task 3 (+ Task 7 obstacle overlay)
      // Solid grid line following on left arena half; RFID-triggered planting.
      // Obstacle check fires on every FOLLOWING iteration.
      // =====================================================================
      case LINED:
        switch (state) {

          case FOLLOWING: {
            // Task 7: obstacle check has priority
            if (obstacleAhead()) {
              stopTracks();
              state = OBSTACLE_DETECTED;
              Serial.println("[Lined] Obstacle detected.");
              break;
            }

            // Task 3: grid line following
            runLineFollower();

            if (checkForHole()) {
              irDetectedAt = millis();
              encoderAtIR  = getTrackEncoder();
              state = WAITING_FOR_RFID;
              Serial.println("[Lined] Hole detected — waiting for RFID.");
            } else if (isJunction() && (getTrackEncoder() - lastJunctionTick) > JUNCTION_COOLDOWN_TICKS) {
              stopTracks();
              state = JUNCTION_HANDLING;
              Serial.println("[Lined] Junction detected.");
            }
            break;
          }

          case JUNCTION_HANDLING: {
            // TODO: implement grid-junction navigation (straight, left, or right
            //       based on planned path / side IR arrays).
            driveStraight(SLOW_SPEED);
            delay(200);
            stopTracks();
            lastJunctionTick = getTrackEncoder();
            resetPID();
            state = FOLLOWING;
            Serial.println("[Lined] Junction cleared.");
            break;
          }

          case WAITING_FOR_RFID: {
            if (isJunction()) {
              driveStraight(SLOW_SPEED);
            } else {
              runLineFollower();
            }

            if (millis() - irDetectedAt > IR_WINDOW_MS) {
              state = FOLLOWING;
              Serial.println("[Lined] RFID timeout — false positive.");
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
              Serial.print("[Lined] RFID: "); Serial.println(detectedUID);
            }
            break;
          }

          case WAITING_FOR_FERTILITY: {
            stopTracks();
            if (millis() - fertilityRequestedAt > FERTILITY_TIMEOUT_MS) {
              state = FOLLOWING;
              Serial.println("[Lined] Fertility timeout — skipping hole.");
            }
            break;
          }

          case DRIVING_TO_HOLE: {
            long tickTarget = encoderAtIR + TICKS_TO_HOLE;
            if ((tickTarget - getTrackEncoder()) > 5) {
              driveStraight(SLOW_SPEED);
            } else {
              stopTracks();
              planterTarget = getPlanterEncoder() + TICKS_TO_PLANT;
              state = PLANTING;
              Serial.println("[Lined] At planting position.");
            }
            break;
          }

          case PLANTING: {
            if (abs(getPlanterEncoder() - planterTarget) > 5) {
              setPlanter(SLOW_SPEED);
            } else {
              setPlanter(0);
              seedPlanted(detectedUID);
              delay(500);
              resetPID();
              state = FOLLOWING;
              Serial.println("[Lined] Seed planted.");
            }
            break;
          }

          case OBSTACLE_DETECTED: {
            // Task 7: determine avoidance direction using side ToF sensors.
            // TODO: read left/right ToF distances and decide which way to turn,
            //       compute turn duration/encoder target, then go to OBSTACLE_AVOIDING.
            state = OBSTACLE_AVOIDING;
            Serial.println("[Lined] Planning avoidance.");
            break;
          }

          case OBSTACLE_AVOIDING: {
            // TODO: execute avoidance manoeuvre (turn, drive past, re-acquire line).
            // Placeholder: wait for obstacle to clear then resume.
            stopTracks();
            if (!obstacleAhead()) {
              resetPID();
              state = FOLLOWING;
              Serial.println("[Lined] Obstacle cleared — resuming.");
            }
            break;
          }

          default:
            state = FOLLOWING;
            break;
        }
        break;

      // =====================================================================
      // BLANK — Task 4 (+ Task 7 obstacle overlay)
      // Right arena half — no floor markings. Navigate between RFID nodes
      // using encoder dead-reckoning. Same RFID/planting logic as LINED.
      // =====================================================================
      case BLANK:
        switch (state) {

          case DEAD_RECKONING: {
            // Task 4: drive TICKS_PER_SQUARE encoder ticks to reach next node.
            // deadReckonTarget must be set before entering this state.
            if (obstacleAhead()) {
              stopTracks();
              state = OBSTACLE_DETECTED;
              Serial.println("[Blank] Obstacle during dead-reckoning.");
              break;
            }

            if ((deadReckonTarget - getTrackEncoder()) > 5) {
              driveStraight(BASE_SPEED);
            } else {
              stopTracks();
              irDetectedAt = millis();
              state = WAITING_FOR_RFID;
              Serial.println("[Blank] Node reached — checking RFID.");
            }
            break;
          }

          case WAITING_FOR_RFID: {
            // Stationary RFID scan at estimated node position
            if (millis() - irDetectedAt > IR_WINDOW_MS) {
              // No tag found — advance to next node
              // TODO: implement path planning to choose next node direction
              deadReckonTarget = getTrackEncoder() + TICKS_PER_SQUARE;
              state = DEAD_RECKONING;
              Serial.println("[Blank] No RFID — advancing to next node.");
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
              fertilityRequestedAt = millis();
              state = WAITING_FOR_FERTILITY;
              Serial.print("[Blank] RFID: "); Serial.println(detectedUID);
            }
            break;
          }

          case WAITING_FOR_FERTILITY: {
            stopTracks();
            if (millis() - fertilityRequestedAt > FERTILITY_TIMEOUT_MS) {
              deadReckonTarget = getTrackEncoder() + TICKS_PER_SQUARE;
              state = DEAD_RECKONING;
              Serial.println("[Blank] Fertility timeout — next node.");
            }
            break;
          }

          case DRIVING_TO_HOLE: {
            long tickTarget = encoderAtIR + TICKS_TO_HOLE;
            if ((tickTarget - getTrackEncoder()) > 5) {
              driveStraight(SLOW_SPEED);
            } else {
              stopTracks();
              planterTarget = getPlanterEncoder() + TICKS_TO_PLANT;
              state = PLANTING;
            }
            break;
          }

          case PLANTING: {
            if (abs(getPlanterEncoder() - planterTarget) > 5) {
              setPlanter(SLOW_SPEED);
            } else {
              setPlanter(0);
              seedPlanted(detectedUID);
              delay(500);
              deadReckonTarget = getTrackEncoder() + TICKS_PER_SQUARE;
              state = DEAD_RECKONING;
              Serial.println("[Blank] Seed planted — next node.");
            }
            break;
          }

          case OBSTACLE_DETECTED: {
            // TODO: same avoidance planning as LINED
            state = OBSTACLE_AVOIDING;
            break;
          }

          case OBSTACLE_AVOIDING: {
            stopTracks();
            if (!obstacleAhead()) {
              deadReckonTarget = getTrackEncoder() + TICKS_PER_SQUARE;
              state = DEAD_RECKONING;
              Serial.println("[Blank] Obstacle cleared.");
            }
            break;
          }

          default:
            deadReckonTarget = getTrackEncoder() + TICKS_PER_SQUARE;
            state = DEAD_RECKONING;
            break;
        }
        break;

      // =====================================================================
      // TUNNEL — Task 5
      // Airlock ramp ascent / descent at reduced speed.
      // TODO: add tilt/IMU detection if available to confirm slope.
      // =====================================================================
      case TUNNEL:
        switch (state) {

          case ASCENDING: {
            // Task 5: climb ramp at SLOW_SPEED to avoid stalling / tipping.
            // TODO: detect ramp end (encoder delta levels off, or front
            //       ultrasonic clears) then transition to next stage.
            driveStraight(SLOW_SPEED);
            Serial.println("[Tunnel] Ascending...");
            break;
          }

          case DESCENDING: {
            // Task 5: controlled descent — cap speed to prevent runaway.
            // TODO: add braking if robot accelerates beyond SLOW_SPEED.
            driveStraight(SLOW_SPEED);
            Serial.println("[Tunnel] Descending...");
            break;
          }

          default:
            state = ASCENDING;
            break;
        }
        break;

      // =====================================================================
      // WALL_FOLLOW — Task 6
      // Maintain WALL_TARGET_MM offset from arena/airlock wall using side
      // ToF sensors. P controller shown; extend to PID as needed.
      // =====================================================================
      case WALL_FOLLOW: {
        // TODO: add getLeftTOFDistance() and getRightTOFDistance() to
        //       sensors.h / sensors.cpp, then replace placeholders below.
        int leftMM = 0;   // placeholder — replace with getLeftTOFDistance()
        // int rightMM = 0; // available if needed for dual-wall following

        float error      = WALL_TARGET_MM - leftMM;
        float correction = Kp_wall * error;
        // TODO: add I and D terms using wallIntegral / wallPrevError

        int leftSpeed  = constrain((int)(BASE_SPEED - correction), MIN_SPEED, MAX_SPEED);
        int rightSpeed = constrain((int)(BASE_SPEED + correction), MIN_SPEED, MAX_SPEED);

        setLeftTrack(leftSpeed);
        setRightTrack(rightSpeed);

        // TODO: exit condition, e.g. end-of-wall via front ultrasonic:
        // if (getUltrasonicDistanceMM() < OBSTACLE_THRESHOLD_MM) { ... }
        break;
      }

      // =====================================================================
      // REVIVAL — Task 8
      // Decelerated approach and stable contact with stranded robot.
      // =====================================================================
      case REVIVAL:
        switch (state) {

          case REVIVAL_APPROACH: {
            // Task 8: slow approach; profile speed as distance closes.
            // TODO: use front ultrasonic to measure distance to stranded robot
            //       and scale speed proportionally rather than fixed encoder target.
            long remaining = revivalApproachTarget - getTrackEncoder();
            if (remaining > 5) {
              driveStraight(REVIVAL_SPEED);
              Serial.println("[Revival] Approaching...");
            } else {
              stopTracks();
              state = REVIVAL_CONTACT;
              Serial.println("[Revival] Contact position reached.");
            }
            break;
          }

          case REVIVAL_CONTACT: {
            // Task 8: confirm contact and trigger revival signal.
            // TODO: implement contact confirmation (bump sensor, motor current
            //       spike, or fixed short nudge) then transition back to mission.
            revive();
            stopTracks();
            // stage = RETURNING; state = FOLLOWING;
            break;
          }

          default:
            state = REVIVAL_APPROACH;
            break;
        }
        break;

      // =====================================================================
      // RETURNING
      // Navigate back to base. Reuse line following on LINED side; switch
      // to wall following when re-entering via airlock perimeter.
      // =====================================================================
      case RETURNING:
        switch (state) {
          case FOLLOWING: {
            runLineFollower();
            // TODO: detect base arrival (junction + RFID home tag) and stop.
            break;
          }
          case WALL_FOLLOWING: {
            // Reuse wall-follow logic inline
            // TODO: same as WALL_FOLLOW stage above once ToF getters exist
            break;
          }
          default:
            state = FOLLOWING;
            break;
        }
        break;

    } // end switch(stage)

  } else if (!killed) {
    rotatePlanter();   // service planter target in manual mode
  }

  // Sensor debug prints — uncomment as needed
  // readTOFSensors();
  // readUltrasonic();
  // readIRArray();
}
