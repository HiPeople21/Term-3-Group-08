/*
 * =====================================================================
 * UCL RAI Robotics Challenge 2026 — Full Autonomous Controller
 * Board  : Arduino GIGA R1 WiFi + Motoron M3S550
 * Version: 2.1 — MFRC522_I2C, dual revive buttons, side QTR arrays
 * =====================================================================
 *
 * REQUIRED LIBRARIES (Sketch → Manage Libraries):
 *   "Motoron"          by Pololu
 *   "QTRSensors"       by Pololu
 *   "Adafruit VL53L0X" by Adafruit
 *   "MFRC522"          → search "MFRC522", install the one with I2C support
 *                        (titled "MFRC522_I2C" or "MFRC522-spi-i2c")
 *                        The WS1850S is register-compatible with RC522.
 *
 * HARDWARE WIRING SUMMARY:
 *   Motoron M3S550     I2C (SDA/SCL)  ch1=Left, ch2=Right, ch3=Planter
 *   QTR-HD-09RC main   pins 22–30     centre IR array (9 sensors)
 *   QTR-HD-02RC left   pins 32, 33    left side IR array  (2 sensors)
 *   QTR-HD-02RC right  pins 34, 35    right side IR array (2 sensors)
 *   HC-SR04            TRIG=5, ECHO=6 front distance
 *   VL53L0X left       I2C XSHUT=7   left side distance  → addr 0x30
 *   VL53L0X right      I2C XSHUT=8   right side distance → addr 0x31
 *   M5Stack RFID2      I2C            addr 0x28 (WS1850S = RC522-compat)
 *   RGB LED (common−)  R=9  G=10 B=11
 *   Kill switch        pin 2          INPUT_PULLUP, active LOW
 *   Revive btn LEFT    pin 3          INPUT_PULLUP, active LOW
 *   Revive btn RIGHT   pin 4          INPUT_PULLUP, active LOW
 *
 * LED BEHAVIOUR:
 *   Solid RED          → alive and running (default / revive-ready state)
 *   Solid GREEN        → revive button pressed (being tapped by ally)
 *   Blinking RED       → kill-switched / stopped
 *   Blue               → calibrating
 *   Other colours      → state indicator (see updateLED())
 *
 * UDP COMMANDS (send to robot IP, port 4210):
 *   "STOP"      — wifi kill switch
 *   "START"     — re-enable
 *   "EMERGENCY" — solar flare! immediate return to base
 * =====================================================================
 */

#include <Wire.h>
#include <WiFi.h>
#include <WiFiUDP.h>
#include <WiFiClient.h>
#include <Motoron.h>
#include <QTRSensors.h>
#include <Adafruit_VL53L0X.h>
#include <MFRC522_I2C.h>   // ← WS1850S is RC522-compatible over I2C

// =====================================================================
//  CONFIGURATION
// =====================================================================
const char*        WIFI_SSID     = "PhaseSpaceNetwork_2.4G";
const char*        WIFI_PASSWORD = "8igMacNet";
const char*        SERVER_IP     = "192.168.1.100";
const int          SERVER_PORT   = 80;
const char*        ROBOT_ID      = "ROBOT_08";
const unsigned int UDP_PORT      = 4210;

// =====================================================================
//  PINS
// =====================================================================
// Kill switch & revive buttons
#define KILL_PIN          2    // Mechanical kill switch
#define REVIVE_BTN_L      3    // Left front revive button
#define REVIVE_BTN_R      4    // Right front revive button

// RGB LED (common cathode — HIGH = on)
#define LED_R             9
#define LED_G             10
#define LED_B             11

// HC-SR04 front ultrasonic
#define TRIG_PIN          5
#define ECHO_PIN          6

// VL53L0X XSHUT (address assignment at boot)
#define TOF_L_XSHUT       7
#define TOF_R_XSHUT       8

// QTR centre array — 9 sensors
const uint8_t QTR_C_PINS[9] = { 22, 23, 24, 25, 26, 27, 28, 29, 30 };
#define QTR_C_COUNT  9

// QTR side arrays — 2 sensors each (QTR-HD-02RC)
const uint8_t QTR_L_PINS[2] = { 32, 33 };   // Left-side array
const uint8_t QTR_R_PINS[2] = { 34, 35 };   // Right-side array
#define QTR_S_COUNT  2

// =====================================================================
//  MOTOR CONFIG
// =====================================================================
#define M_LEFT    1
#define M_RIGHT   2
#define M_PLANT   3
#define L_DIR     1    // flip to -1 if left motor runs backwards
#define R_DIR     1    // flip to -1 if right motor runs backwards
#define MAX_SPD   800

// ★ CALIBRATE on your robot ★
#define BASE_SPD     350
#define TURN_SPD     300
#define PLANT_SPD    500
#define PLANT_MS     650
#define TURN_90_MS   650   // ★ tune by timing a physical 90° turn
#define UTURN_MS     1300  // ★ tune by timing a physical 180° turn

// =====================================================================
//  ★ LINE FOLLOWING PID ★
//  Tune order: fix KP first (no KI/KD), then add KD, KI last.
//  Too much KP → oscillates.  Too little KP → sluggish correction.
// =====================================================================
const float LF_KP = 0.35f;
const float LF_KI = 0.0005f;
const float LF_KD = 0.15f;
const int   LF_CTR      = 4000;   // Centre position (0–8000 for 9 sensors)
const int   LF_MAX_CORR = 350;    // Max ±correction added to each motor

// Side-sensor correction weight (blended into PID output)
// If side sensors see black while centre sensors are near-centre, nudge away
const int   SIDE_CORR   = 60;

// How many of the inner 5 centre sensors must read black for "intersection"
#define INTERSECT_MIN 4
// RC timing value above which a sensor counts as "black"
#define QTR_BLACK  300  // ★ measure over your actual floor — adjust if needed

// =====================================================================
//  ★ WALL FOLLOWING ★
// =====================================================================
const float WF_KP        = 3.0f;
const float WF_TARGET_CM = 15.0f;   // ★ target distance from left wall
const int   WF_MAX_CORR  = 200;
const float WF_STOP_CM   = 12.0f;   // Front obstacle → turn

// =====================================================================
//  TIMING / LIMITS
// =====================================================================
const unsigned long ARENA_MS       = 5UL * 60UL * 1000UL;
const unsigned long CHARGE_MS      = 5UL * 60UL * 1000UL;
const unsigned long HTTP_TIMEOUT   = 3000;
const unsigned long DOOR_TIMEOUT   = 8000;
const unsigned long ENTRY_RETRY_MS = 2000;
const int           MAX_SEEDS      = 5;

// =====================================================================
//  STATE MACHINE
// =====================================================================
enum State {
  S_IDLE, S_CALIBRATE,
  S_BASE_TO_EXIT, S_REQUEST_EXIT, S_WAIT_DOOR, S_EXIT_TUNNEL,
  S_EXPLORE_LINE, S_EXPLORE_WALL,
  S_QUERY_SERVER, S_PLANT,
  S_RETURN_TO_ENTRY, S_REQUEST_ENTRY, S_ENTER_TUNNEL,
  S_BASE_TO_PARK, S_CHARGING,
  S_EMERGENCY_RETURN, S_ERROR
};
const char* STATE_NAMES[] = {
  "IDLE","CALIBRATE","BASE_TO_EXIT","REQUEST_EXIT","WAIT_DOOR",
  "EXIT_TUNNEL","EXPLORE_LINE","EXPLORE_WALL","QUERY_SERVER","PLANT",
  "RETURN_TO_ENTRY","REQUEST_ENTRY","ENTER_TUNNEL","BASE_TO_PARK",
  "CHARGING","EMERGENCY_RETURN","ERROR"
};

State gState        = S_IDLE;
State gPrevState    = S_IDLE;

// =====================================================================
//  OBJECTS
// =====================================================================
MotoronI2C       motoron;
QTRSensors       qtrC;          // Centre array (9 sensors)
QTRSensors       qtrL;          // Left side array (2 sensors)
QTRSensors       qtrR;          // Right side array (2 sensors)
Adafruit_VL53L0X tofL, tofR;
WiFiUDP          udp;
MFRC522          rfid(0x28, UINT8_MAX);  // addr=0x28, resetPin unused (I2C)

// =====================================================================
//  GLOBAL STATE
// =====================================================================
bool  gEnabled       = false;
bool  gPrevBtn       = false;
bool  gWifiKill      = false;
bool  gEmergency     = false;
bool  gLineMode      = true;
bool  gEmergUTurn    = false;

int   gSeedsLeft     = MAX_SEEDS;
int   gSeedsPlanted  = 0;
char  gTagUID[24]    = {0};

unsigned long gArenaStart   = 0;
unsigned long gStateStart   = 0;
unsigned long gLastDebounce = 0;
unsigned long gLastBlink    = 0;
bool          gBlinkOn      = false;
unsigned long gLineLostAt   = 0;

// PID
float         gPidI     = 0;
int           gPidEPrev = 0;
unsigned long gPidT     = 0;

// TOF last-known (continuous mode)
float gTofLLast = 30.0f;
float gTofRLast = 30.0f;

#define TOF_L_ADDR  0x30
#define TOF_R_ADDR  0x31

// =====================================================================
//  setup()
// =====================================================================
void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000);
  Serial.println(F("\n=== UCL RAI Robot v2.1 ==="));

  Wire.begin();

  // LED
  pinMode(LED_R, OUTPUT); pinMode(LED_G, OUTPUT); pinMode(LED_B, OUTPUT);
  rgb(255, 0, 0);

  // Buttons
  pinMode(KILL_PIN,     INPUT_PULLUP);
  pinMode(REVIVE_BTN_L, INPUT_PULLUP);
  pinMode(REVIVE_BTN_R, INPUT_PULLUP);

  // HC-SR04
  pinMode(TRIG_PIN, OUTPUT); pinMode(ECHO_PIN, INPUT);
  digitalWrite(TRIG_PIN, LOW);

  initMotors();
  initQTR();
  initTOF();
  initRFID();
  initWiFi();

  enterState(S_IDLE);
  Serial.println(F("Ready — press kill switch to start."));
}

// =====================================================================
//  loop()
// =====================================================================
void loop() {
  checkKillSwitch();
  checkUDP();
  checkReviveButtons();   // Always handle — works even when stopped

  if (!gEnabled) {
    stopAll();
    blinkRGB(255, 0, 0, 300);
    return;
  }

  // Emergency overrides active exploration states
  if (gEmergency && gState != S_EMERGENCY_RETURN
                 && gState != S_REQUEST_ENTRY
                 && gState != S_ENTER_TUNNEL
                 && gState != S_BASE_TO_PARK
                 && gState != S_CHARGING) {
    enterState(S_EMERGENCY_RETURN);
  }

  // Arena time limit
  if (gArenaStart > 0 && isInArena() &&
      millis() - gArenaStart > ARENA_MS) {
    Serial.println(F("[TIMER] Arena time up."));
    enterState(S_RETURN_TO_ENTRY);
  }

  runState();
}

bool isInArena() {
  return gState == S_EXPLORE_LINE || gState == S_EXPLORE_WALL  ||
         gState == S_QUERY_SERVER  || gState == S_PLANT        ||
         gState == S_RETURN_TO_ENTRY;
}

// =====================================================================
//  REVIVE BUTTONS
//  Priority: kill-switch blink > green (tapped) > normal state LED
// =====================================================================
void checkReviveButtons() {
  bool tapped = (digitalRead(REVIVE_BTN_L) == LOW) ||
                (digitalRead(REVIVE_BTN_R) == LOW);

  if (tapped && gEnabled) {
    // Immediately override LED to green — visible even if state LED is different
    rgb(0, 255, 0);
  }
  // When not tapped and enabled, LED is managed by updateLED() / blinkRGB()
  // The kill-switch blink (in loop()) takes priority when gEnabled==false
}

// =====================================================================
//  STATE MACHINE RUNNER
// =====================================================================
void runState() {
  switch (gState) {

    case S_IDLE:
      stopAll();
      break;

    case S_CALIBRATE:
      calibrateQTR();
      enterState(S_BASE_TO_EXIT);
      break;

    // ── Navigate inside base to RFID tag B ──────────────────────────
    case S_BASE_TO_EXIT:
      if (lineFollow() == LF_INTERSECTION) {
        stopAll(); delay(100);
        if (pollRFID()) enterState(S_REQUEST_EXIT);
      }
      break;

    // ── Request exit clearance via server ───────────────────────────
    case S_REQUEST_EXIT: {
      stopAll();
      bool ok = httpGetContains(
        "/api/exit?robot=" + String(ROBOT_ID) + "&rfid=" + String(gTagUID),
        "granted");
      if (ok) { Serial.println(F("[EXIT] Granted.")); enterState(S_WAIT_DOOR); }
      else    { Serial.println(F("[EXIT] Denied — retry.")); delay(2000); }
      break;
    }

    // ── Wait for Tunnel B door to open ──────────────────────────────
    case S_WAIT_DOOR: {
      float f = frontDist();
      if (f < 0 || f > 20.0f || millis() - gStateStart > DOOR_TIMEOUT) {
        Serial.println(F("[DOOR] Open — entering tunnel."));
        enterState(S_EXIT_TUNNEL);
      } else {
        driveForward(150);  // Creep forward to trigger door sensor
      }
      break;
    }

    // ── Drive up ramp through Tunnel B ──────────────────────────────
    case S_EXIT_TUNNEL:
      tunnelFollow();
      if (millis() - gStateStart > 3500) {
        gArenaStart = millis();
        gLineMode   = true;
        gLineLostAt = 0;
        enterState(S_EXPLORE_LINE);
      }
      break;

    // ── Arena: PID line following (left half) ───────────────────────
    case S_EXPLORE_LINE: {
      if (lineLost()) {
        if (gLineLostAt == 0) gLineLostAt = millis();
        if (millis() - gLineLostAt > 1000) {
          gLineLostAt = 0;
          gLineMode   = false;
          Serial.println(F("[NAV] Line lost — wall follow."));
          enterState(S_EXPLORE_WALL);
        }
        driveForward(BASE_SPD);  // Coast briefly during line-loss window
        break;
      }
      gLineLostAt = 0;

      if (pollRFID()) { stopAll(); enterState(S_QUERY_SERVER); break; }

      if (lineFollow() == LF_INTERSECTION) {
        stopAll(); delay(150);
        if (pollRFID()) enterState(S_QUERY_SERVER);
        else            gridAdvance();
      }
      break;
    }

    // ── Arena: wall following (right half, no lines) ─────────────────
    case S_EXPLORE_WALL:
      wallFollow();
      if (pollRFID()) { stopAll(); enterState(S_QUERY_SERVER); break; }
      if (!lineLost()) { gLineMode = true; enterState(S_EXPLORE_LINE); }
      break;

    // ── Stopped at RFID tag — ask server about soil ──────────────────
    case S_QUERY_SERVER: {
      stopAll();
      if (gSeedsLeft <= 0) { enterState(exploreState()); break; }
      bool fertile = httpGetContains("/api/soil?id=" + String(gTagUID), "fertile");
      Serial.print(F("[SOIL] ")); Serial.println(fertile ? F("FERTILE") : F("infertile"));
      enterState(fertile ? S_PLANT : exploreState());
      break;
    }

    // ── Deposit one seed ─────────────────────────────────────────────
    case S_PLANT:
      plantSeed();
      gSeedsLeft--; gSeedsPlanted++;
      Serial.print(F("[PLANT] ")); Serial.print(gSeedsPlanted);
      Serial.print(F(" planted, ")); Serial.print(gSeedsLeft); Serial.println(F(" left."));
      enterState(gSeedsLeft <= 0 ? S_RETURN_TO_ENTRY : exploreState());
      break;

    // ── Head back across arena toward Tunnel A ───────────────────────
    case S_RETURN_TO_ENTRY: {
      static bool uDone = false;
      if (gPrevState != S_RETURN_TO_ENTRY) uDone = false;
      if (!uDone) { uTurn(); uDone = true; pidReset(); }

      if (gLineMode) lineFollow(); else wallFollow();
      if (pollRFID()) { stopAll(); enterState(S_REQUEST_ENTRY); }
      break;
    }

    // ── Request entry clearance (base robot must open door) ──────────
    case S_REQUEST_ENTRY: {
      stopAll();
      bool ok = httpGetContains("/api/enter?robot=" + String(ROBOT_ID), "granted");
      if (ok) { gEmergency = false; enterState(S_ENTER_TUNNEL); }
      else    { Serial.println(F("[ENTRY] Waiting...")); delay(ENTRY_RETRY_MS); }
      break;
    }

    // ── Drive down ramp through Tunnel A ─────────────────────────────
    case S_ENTER_TUNNEL:
      tunnelFollow();
      if (millis() - gStateStart > 4000) { gArenaStart = 0; enterState(S_BASE_TO_PARK); }
      break;

    // ── Navigate inside base to parking area ─────────────────────────
    case S_BASE_TO_PARK:
      if (lineFollow() == LF_LINE_END) {
        stopAll();
        Serial.println(F("[PARK] Parked."));
        enterState(S_CHARGING);
      }
      break;

    // ── Charging (wait in base before next run) ───────────────────────
    case S_CHARGING:
      stopAll();
      if (millis() - gStateStart > CHARGE_MS) {
        gSeedsLeft = MAX_SEEDS;
        gLineMode  = true;
        enterState(S_BASE_TO_EXIT);
      }
      break;

    // ── Emergency return (solar flare) ───────────────────────────────
    case S_EMERGENCY_RETURN:
      if (!gEmergUTurn) { uTurn(); gEmergUTurn = true; pidReset(); }
      if (gLineMode) lineFollowFast(); else wallFollow();
      if (pollRFID()) { stopAll(); enterState(S_REQUEST_ENTRY); }
      break;

    case S_ERROR:
      stopAll(); rgb(255, 0, 0); break;
  }
}

// =====================================================================
//  STATE HELPERS
// =====================================================================
void enterState(State s) {
  gPrevState  = gState;
  gState      = s;
  gStateStart = millis();
  if (s == S_EMERGENCY_RETURN) gEmergUTurn = false;
  pidReset();
  Serial.print(F("[STATE] → ")); Serial.println(STATE_NAMES[s]);
  updateLED();
}

State exploreState() { return gLineMode ? S_EXPLORE_LINE : S_EXPLORE_WALL; }

// =====================================================================
//  LINE FOLLOWING — PID with side-array correction
// =====================================================================
enum LineResult { LF_FOLLOWING, LF_INTERSECTION, LF_LINE_END };

LineResult lineFollow()     { return _lf(BASE_SPD);        }
LineResult lineFollowFast() { return _lf(BASE_SPD + 80);   }

LineResult _lf(int spd) {
  uint16_t cv[QTR_C_COUNT];
  uint16_t pos = qtrC.readLineBlack(cv);   // 0–8000, centre=4000

  // Intersection check (inner 5 sensors mostly black)
  int nBlack = 0;
  for (int i = 2; i <= 6; i++) if (cv[i] > QTR_BLACK) nBlack++;
  if (nBlack >= INTERSECT_MIN) return LF_INTERSECTION;

  // Line-end check (all sensors white)
  int anyB = 0;
  for (int i = 0; i < QTR_C_COUNT; i++) if (cv[i] > QTR_BLACK) anyB++;
  if (anyB == 0) return LF_LINE_END;

  // ── PID ─────────────────────────────────────────────────────────
  unsigned long now = millis();
  float dt = (gPidT == 0) ? 0.02f : (now - gPidT) / 1000.0f;
  if (dt > 0.1f) dt = 0.1f;
  gPidT = now;

  int err = (int)pos - LF_CTR;
  gPidI += err * dt;
  gPidI = constrain(gPidI, -3000.0f, 3000.0f);   // Anti-windup
  float deriv = (err - gPidEPrev) / dt;
  gPidEPrev = err;

  float out = LF_KP * err + LF_KI * gPidI + LF_KD * deriv;

  // ── Side array correction ────────────────────────────────────────
  // If left side sensor sees black → robot drifted left → nudge right
  // If right side sensor sees black → robot drifted right → nudge left
  uint16_t lv[QTR_S_COUNT], rv[QTR_S_COUNT];
  qtrL.read(lv); qtrR.read(rv);
  bool leftBlack  = (lv[0] > QTR_BLACK || lv[1] > QTR_BLACK);
  bool rightBlack = (rv[0] > QTR_BLACK || rv[1] > QTR_BLACK);

  if (leftBlack  && !rightBlack) out += SIDE_CORR;   // Nudge right
  if (rightBlack && !leftBlack)  out -= SIDE_CORR;   // Nudge left

  out = constrain(out, (float)-LF_MAX_CORR, (float)LF_MAX_CORR);

  drive(constrain(spd - (int)out, -MAX_SPD, MAX_SPD),
        constrain(spd + (int)out, -MAX_SPD, MAX_SPD));
  return LF_FOLLOWING;
}

bool lineLost() {
  uint16_t v[QTR_C_COUNT];
  qtrC.read(v);
  for (int i = 0; i < QTR_C_COUNT; i++) if (v[i] > QTR_BLACK) return false;
  return true;
}

void pidReset() { gPidI = 0; gPidEPrev = 0; gPidT = 0; }

void calibrateQTR() {
  Serial.println(F("[CAL] Calibrating — drive robot over line for 2 s..."));
  rgb(0, 0, 255);
  unsigned long end = millis() + 2000;
  while (millis() < end) {
    driveForward(180);
    qtrC.calibrate();
    qtrL.calibrate();
    qtrR.calibrate();
    delay(5);
  }
  stopAll();
  Serial.println(F("[CAL] Done."));
}

// =====================================================================
//  GRID TURN — snake pattern, alternating direction each row
// =====================================================================
static bool gGridFwd = true;

void gridAdvance() {
  if (gGridFwd) {
    turn90Right(); driveForward(BASE_SPD); delay(320); stopAll(); turn90Right();
  } else {
    turn90Left();  driveForward(BASE_SPD); delay(320); stopAll(); turn90Left();
  }
  gGridFwd = !gGridFwd;
  pidReset();
}

// =====================================================================
//  WALL FOLLOWING
// =====================================================================
void wallFollow() {
  float l = readTOFL();
  float r = readTOFR();
  float f = frontDist();

  // Obstacle ahead — turn away from left wall
  if (f > 0 && f < WF_STOP_CM) { stopAll(); delay(100); turn90Right(); return; }

  // Also check side QTR arrays for close obstacles/walls not in TOF range
  uint16_t lv[QTR_S_COUNT], rv[QTR_S_COUNT];
  qtrL.read(lv); qtrR.read(rv);
  bool leftClose  = (lv[0] > QTR_BLACK || lv[1] > QTR_BLACK);
  bool rightClose = (rv[0] > QTR_BLACK || rv[1] > QTR_BLACK);

  float err;
  if      (l > 0)    err =  l - WF_TARGET_CM;
  else if (r > 0)    err = -(r - WF_TARGET_CM);
  else               { driveForward(BASE_SPD); return; }

  // Side QTR augments the TOF error signal
  if (leftClose  && !rightClose) err += 5.0f;   // Too close left → steer right
  if (rightClose && !leftClose)  err -= 5.0f;   // Too close right → steer left

  int corr = constrain((int)(WF_KP * err), -WF_MAX_CORR, WF_MAX_CORR);
  drive(constrain(BASE_SPD + corr, 0, MAX_SPD),
        constrain(BASE_SPD - corr, 0, MAX_SPD));
}

void tunnelFollow() {
  float l = readTOFL(), r = readTOFR();
  float err = (l > 0 && r > 0) ? (l - r) :
              (l > 0)           ? (l - WF_TARGET_CM) :
              (r > 0)           ? -(r - WF_TARGET_CM) : 0;
  int corr = constrain((int)(WF_KP * err), -WF_MAX_CORR, WF_MAX_CORR);
  drive(constrain(BASE_SPD + corr, 0, MAX_SPD),
        constrain(BASE_SPD - corr, 0, MAX_SPD));
}

// =====================================================================
//  TOF SENSORS — continuous mode for fast non-blocking reads
// =====================================================================
float readTOFL() {
  if (tofL.isRangeComplete()) {
    uint16_t mm = tofL.readRangeResult();
    if (mm < 8190) gTofLLast = mm / 10.0f;
  }
  return gTofLLast;
}

float readTOFR() {
  if (tofR.isRangeComplete()) {
    uint16_t mm = tofR.readRangeResult();
    if (mm < 8190) gTofRLast = mm / 10.0f;
  }
  return gTofRLast;
}

// =====================================================================
//  HC-SR04 FRONT DISTANCE
// =====================================================================
float frontDist() {
  digitalWrite(TRIG_PIN, LOW);  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH); delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  long dur = pulseIn(ECHO_PIN, HIGH, 30000UL);
  return (dur == 0) ? -1.0f : (dur * 0.0343f) / 2.0f;
}

// =====================================================================
//  RFID — WS1850S via MFRC522_I2C library
// =====================================================================
bool pollRFID() {
  if (!rfid.PICC_IsNewCardPresent()) return false;
  if (!rfid.PICC_ReadCardSerial())   return false;

  gTagUID[0] = '\0';
  char tmp[4];
  for (uint8_t i = 0; i < rfid.uid.size; i++) {
    if (i > 0) strcat(gTagUID, ":");
    snprintf(tmp, sizeof(tmp), "%02X", rfid.uid.uidByte[i]);
    strcat(gTagUID, tmp);
  }
  Serial.print(F("[RFID] UID: ")); Serial.println(gTagUID);
  rfid.PICC_HaltA();
  return true;
}

// =====================================================================
//  SERVER HTTP GET
// =====================================================================
bool httpGetContains(const String& path, const char* keyword) {
  WiFiClient client;
  if (!client.connect(SERVER_IP, SERVER_PORT)) {
    Serial.println(F("[HTTP] Cannot reach server.")); return false;
  }
  client.print(F("GET ")); client.print(path); client.println(F(" HTTP/1.1"));
  client.print(F("Host: ")); client.println(SERVER_IP);
  client.println(F("Connection: close")); client.println();

  unsigned long t = millis();
  while (!client.available() && millis() - t < HTTP_TIMEOUT);
  if (!client.available()) { client.stop(); return false; }

  // Skip HTTP headers
  while (client.connected() || client.available()) {
    String line = client.readStringUntil('\n');
    if (line == "\r" || line.length() <= 1) break;
  }
  String body = client.readString();
  client.stop();
  Serial.print(F("[HTTP] ")); Serial.println(body);
  return body.indexOf(keyword) >= 0;
}

// =====================================================================
//  KILL SWITCH — MECHANICAL
// =====================================================================
void checkKillSwitch() {
  bool pressed = (digitalRead(KILL_PIN) == LOW);
  if (pressed && !gPrevBtn && millis() - gLastDebounce > 50) {
    gLastDebounce = millis();
    if (gState == S_IDLE && !gEnabled) {
      gEnabled = true; enterState(S_CALIBRATE);
    } else {
      gEnabled = !gEnabled;
      if (gEnabled) { Serial.println(F("[KILL-HW] ON")); updateLED(); }
      else          { stopAll(); Serial.println(F("[KILL-HW] OFF")); }
    }
  }
  gPrevBtn = pressed;
}

// =====================================================================
//  KILL SWITCH — UDP
// =====================================================================
void checkUDP() {
  if (WiFi.status() != WL_CONNECTED) return;
  int n = udp.parsePacket(); if (n <= 0) return;
  char buf[32] = {0}; udp.read(buf, 31);
  String cmd = String(buf); cmd.trim();

  if      (cmd.equalsIgnoreCase("STOP"))      { gEnabled = false; gWifiKill = true; stopAll(); Serial.println(F("[UDP] STOPPED")); }
  else if (cmd.equalsIgnoreCase("START"))     { gWifiKill = false; gEnabled = true; Serial.println(F("[UDP] ENABLED")); updateLED(); }
  else if (cmd.equalsIgnoreCase("EMERGENCY")) { gEmergency = true; Serial.println(F("[UDP] EMERGENCY")); }
}

// =====================================================================
//  LED
// =====================================================================
void rgb(uint8_t r, uint8_t g, uint8_t b) {
  analogWrite(LED_R, r); analogWrite(LED_G, g); analogWrite(LED_B, b);
}

void blinkRGB(uint8_t r, uint8_t g, uint8_t b, uint32_t period) {
  if (millis() - gLastBlink >= period) {
    gLastBlink = millis(); gBlinkOn = !gBlinkOn;
    rgb(gBlinkOn ? r : 0, gBlinkOn ? g : 0, gBlinkOn ? b : 0);
  }
}

void updateLED() {
  // Default: solid red (robot alive, ready to be tapped)
  // checkReviveButtons() overrides to green when a button is pressed
  switch (gState) {
    case S_IDLE:               rgb(255,  80,  0);   break;  // Amber
    case S_CALIBRATE:          rgb(  0,   0,255);   break;  // Blue
    case S_BASE_TO_EXIT:
    case S_BASE_TO_PARK:       rgb(255, 255,  0);   break;  // Yellow
    case S_EXPLORE_LINE:       rgb(  0, 255,  0);   break;  // Green
    case S_EXPLORE_WALL:       rgb(  0, 200,100);   break;  // Teal
    case S_QUERY_SERVER:       rgb(  0, 255,255);   break;  // Cyan
    case S_PLANT:              rgb(128,   0,255);   break;  // Purple
    case S_RETURN_TO_ENTRY:
    case S_EMERGENCY_RETURN:   rgb(255,   0,  0);   break;  // Red
    case S_REQUEST_ENTRY:      rgb(255, 165,  0);   break;  // Orange
    case S_CHARGING:           rgb(255, 100,  0);   break;  // Warm amber
    default:                   rgb(255,   0,  0);   break;  // Red default
  }
}

// =====================================================================
//  MOTORS
// =====================================================================
void drive(int l, int r) {
  if (!gEnabled) { stopAll(); return; }
  motoron.setSpeed(M_LEFT,  constrain(l * L_DIR, -MAX_SPD, MAX_SPD));
  motoron.setSpeed(M_RIGHT, constrain(r * R_DIR, -MAX_SPD, MAX_SPD));
}
void stopAll() {
  motoron.setSpeedNow(M_LEFT, 0); motoron.setSpeedNow(M_RIGHT, 0);
  motoron.setSpeedNow(M_PLANT, 0);
}
void driveForward (int s) { drive( s,  s); }
void driveBackward(int s) { drive(-s, -s); }
void pivotL       (int s) { drive(-s,  s); }
void pivotR       (int s) { drive( s, -s); }
void turn90Left()  { pivotL(TURN_SPD); delay(TURN_90_MS); stopAll(); }
void turn90Right() { pivotR(TURN_SPD); delay(TURN_90_MS); stopAll(); }
void uTurn()       { pivotR(TURN_SPD); delay(UTURN_MS);   stopAll(); }

void plantSeed() {
  Serial.println(F("[PLANT] Depositing..."));
  motoron.setSpeed(M_PLANT, PLANT_SPD);
  delay(PLANT_MS);
  motoron.setSpeedNow(M_PLANT, 0);
}

// =====================================================================
//  INIT
// =====================================================================
void initMotors() {
  motoron.reinitialize(); motoron.disableCRC();
  motoron.clearResetFlag(); motoron.disableCommandTimeout();
  for (uint8_t ch = 1; ch <= 3; ch++) {
    motoron.setMaxAcceleration(ch, 200); motoron.setMaxDeceleration(ch, 300);
  }
  stopAll();
  Serial.println(F("[Motors]  OK"));
}

void initQTR() {
  // Centre array
  qtrC.setTypeRC(); qtrC.setSensorPins(QTR_C_PINS, QTR_C_COUNT);
  // Side arrays
  qtrL.setTypeRC(); qtrL.setSensorPins(QTR_L_PINS, QTR_S_COUNT);
  qtrR.setTypeRC(); qtrR.setSensorPins(QTR_R_PINS, QTR_S_COUNT);
  Serial.println(F("[QTR]     OK  centre(9) + left(2) + right(2)"));
}

void initTOF() {
  pinMode(TOF_L_XSHUT, OUTPUT); pinMode(TOF_R_XSHUT, OUTPUT);
  digitalWrite(TOF_L_XSHUT, LOW); digitalWrite(TOF_R_XSHUT, LOW); delay(10);

  digitalWrite(TOF_L_XSHUT, HIGH); delay(10);
  if (!tofL.begin(TOF_L_ADDR)) Serial.println(F("[TOF L]   FAILED"));
  else { tofL.startRangeContinuous(20); Serial.print(F("[TOF L]   OK @ 0x")); Serial.println(TOF_L_ADDR, HEX); }

  digitalWrite(TOF_R_XSHUT, HIGH); delay(10);
  if (!tofR.begin(TOF_R_ADDR)) Serial.println(F("[TOF R]   FAILED"));
  else { tofR.startRangeContinuous(20); Serial.print(F("[TOF R]   OK @ 0x")); Serial.println(TOF_R_ADDR, HEX); }
}

void initRFID() {
  rfid.PCD_Init();
  Serial.print(F("[RFID]    WS1850S (MFRC522-compat) firmware 0x"));
  Serial.println(rfid.PCD_ReadRegister(rfid.VersionReg), HEX);
}

void initWiFi() {
  Serial.print(F("[WiFi]    Connecting to ")); Serial.print(WIFI_SSID);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  unsigned long t = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - t < 10000UL) { delay(500); Serial.print('.'); }
  if (WiFi.status() == WL_CONNECTED) {
    Serial.print(F("\n[WiFi]    IP: ")); Serial.println(WiFi.localIP());
    udp.begin(UDP_PORT);
    Serial.print(F("[UDP]     Port ")); Serial.println(UDP_PORT);
  } else {
    Serial.println(F("\n[WiFi]    FAILED."));
  }
}
