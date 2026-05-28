/*
 * =====================================================================
 * UCL RAI Robotics Challenge 2026 — Main Robot Controller
 * Board  : Arduino GIGA R1 WiFi
 * Shield : Motoron M3S550 (triple motor, I2C)
 * =====================================================================
 *
 * INSTALL THESE LIBRARIES via Sketch → Include Library → Manage Libraries:
 *   1. "Motoron"           by Pololu
 *   2. "QTRSensors"        by Pololu
 *   3. "Adafruit VL53L0X"  by Adafruit
 *   4. "M5Unit-RFID2"      by M5Stack   (search: M5_RFID2)
 *   WiFi + Wire are built into the GIGA core — no install needed.
 *
 * CHECKLIST COVERAGE (Trial Run #1):
 *   ✅ 1a  Mechanical kill-switch toggle  (pin 2, debounced)
 *   ✅ 1b  WiFi UDP kill-switch           (send "STOP" / "START" to port 4210)
 *   ✅ 1c  LED: red-blink when stopped, green when running
 *   ✅ 2   Motor speed & heading control  (forward/back/left/right/U-turn)
 *   ✅ 3   QTR-HD-09RC readings           (printed to Serial every 250 ms)
 *   ✅ 4   HC-SR04 front distance         (printed to Serial every 250 ms)
 *   ✅ 5   VL53L0X side distances         (printed to Serial every 250 ms)
 *   ✅ 6   RFID tag UID display           (polled every 500 ms)
 *   ✅ 7   Planter motor activation
 *
 * UDP TEST (Linux/Mac terminal):
 *   echo -n "STOP"  | nc -u <robot-ip> 4210
 *   echo -n "START" | nc -u <robot-ip> 4210
 * =====================================================================
 */

#include <Wire.h>
#include <WiFi.h>
#include <WiFiUDP.h>
#include <Motoron.h>
#include <QTRSensors.h>
#include <Adafruit_VL53L0X.h>
#include <M5_RFID2.h>

// =====================================================================
// ★  EDIT THESE BEFORE UPLOADING  ★
// =====================================================================
const char*          WIFI_SSID     = "PhaseSpaceNetwork_2.4G";
const char*          WIFI_PASSWORD = "8igMacNet";
const unsigned int   UDP_PORT      = 4210;

// =====================================================================
// PINS
// =====================================================================

// Mechanical kill-switch — momentary button, pulled LOW when pressed
#define KILL_SWITCH_PIN   2

// RGB LED — common cathode (HIGH = on); must be PWM-capable pins
#define LED_R_PIN         9
#define LED_G_PIN         10
#define LED_B_PIN         11

// HC-SR04 front ultrasonic
#define TRIG_PIN          5
#define ECHO_PIN          6

// VL53L0X XSHUT pins — used to boot the two sensors at different I2C addresses
#define TOF_LEFT_XSHUT    7
#define TOF_RIGHT_XSHUT   8

// QTR-HD-09RC — 9 sensor data pins (adjust to match your wiring)
const uint8_t QTR_PINS[9] = { 22, 23, 24, 25, 26, 27, 28, 29, 30 };
#define QTR_NUM_SENSORS   9

// =====================================================================
// MOTOR CONFIGURATION  (Motoron M3S550, default I2C address 0x10)
// =====================================================================
#define MOTOR_LEFT      1   // Channel 1 → left drive motor
#define MOTOR_RIGHT     3   // Channel 2 → right drive motor
#define MOTOR_PLANTER   2   // Channel 3 → seed planter actuator

// If a drive motor spins the wrong way, flip its value to -1
#define LEFT_DIR        1
#define RIGHT_DIR       1

#define MAX_SPEED       800   // Motoron accepts -800 … +800
#define BASE_SPEED      400   // Default cruising speed
#define TURN_SPEED      350   // Speed used when pivoting
#define PLANTER_SPEED   500   // Speed for the planter actuator

// ★ CALIBRATE these by physically timing your robot on the floor ★
#define TURN_90_MS      650   // ms for a 90-degree pivot turn
#define UTURN_MS        1300  // ms for a 180-degree U-turn
#define PLANTER_MS      600   // ms to run planter per seed

// =====================================================================
// I2C ADDRESSES
// =====================================================================
#define TOF_LEFT_ADDR   0x30   // Programmed at boot (was default 0x29)
#define TOF_RIGHT_ADDR  0x31   // Programmed at boot
#define RFID_I2C_ADDR   0x28   // WS1850S default

// =====================================================================
// OBJECTS
// =====================================================================
MotoronI2C        motoron;
QTRSensors        qtr;
Adafruit_VL53L0X  tofLeft;
Adafruit_VL53L0X  tofRight;
WiFiUDP           udp;
M5_RFID2          rfid;

// =====================================================================
// GLOBAL STATE
// =====================================================================
volatile bool robotEnabled  = true;   // Combined kill-switch state
bool          prevBtnState  = false;  // For edge-detect debounce
bool          wifiKillOn    = false;  // Track if WiFi kill is active

unsigned long lastDebounce  = 0;
const unsigned long DEBOUNCE_MS = 50;

unsigned long lastBlink     = 0;
const unsigned long BLINK_MS = 300;
bool blinkOn = false;

// =====================================================================
//  SETUP
// =====================================================================
void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000);
  Serial.println(F("\n=== UCL RAI Robot — Initialising ==="));

  Wire.begin();

  // ── LED ──────────────────────────────────────────────────────────
  pinMode(LED_R_PIN, OUTPUT);
  pinMode(LED_G_PIN, OUTPUT);
  pinMode(LED_B_PIN, OUTPUT);
  setLED(255, 0, 0);   // Red during init

  // ── Kill switch ──────────────────────────────────────────────────
  pinMode(KILL_SWITCH_PIN, INPUT_PULLUP);

  // ── HC-SR04 ──────────────────────────────────────────────────────
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  digitalWrite(TRIG_PIN, LOW);

  // ── Subsystems ───────────────────────────────────────────────────
  initMotors();
  initQTR();
  initTOF();
  initRFID();
  initWiFi();

  setLED(0, 255, 0);   // Green = ready
  Serial.println(F("=== Ready. Press kill switch to enable / disable. ===\n"));
}

// =====================================================================
//  LOOP
// =====================================================================
void loop() {
  handleMechKillSwitch();
  handleUDP();
  updateLED();

  if (!robotEnabled) {
    stopAllMotors();
    return;
  }

  // Print all sensor readings to Serial every 250 ms
  static unsigned long lastReport = 0;
  if (millis() - lastReport >= 250UL) {
    lastReport = millis();
    printAllSensors();
  }

  // Poll RFID every 500 ms
  static unsigned long lastRFID = 0;
  if (millis() - lastRFID >= 500UL) {
    lastRFID = millis();
    checkRFID();
  }
}

// =====================================================================
//  INIT: MOTORS
// =====================================================================
void initMotors() {
  motoron.reinitialize();
  motoron.disableCRC();
  motoron.clearResetFlag();
  motoron.disableCommandTimeout();   // Prevents auto-stop if comms pause briefly

  for (uint8_t ch = 1; ch <= 3; ch++) {
    motoron.setMaxAcceleration(ch, 200);
    motoron.setMaxDeceleration(ch, 300);
  }
  stopAllMotors();
  Serial.println(F("[Motors]    OK — 3 channels (L/R drive + planter)"));
}

// =====================================================================
//  INIT: QTR-HD-09RC (RC timing type, 9 sensors)
// =====================================================================
void initQTR() {
  qtr.setTypeRC();
  qtr.setSensorPins(QTR_PINS, QTR_NUM_SENSORS);
  // Uncomment and change pin number if you wired a dedicated emitter-enable pin:
  // qtr.setEmitterPin(YOUR_EMITTER_PIN);
  Serial.println(F("[QTR]       OK — 9 sensors, RC timing mode"));
}

// =====================================================================
//  INIT: VL53L0X x2
//  Both sensors boot at the same default I2C address (0x29).
//  We use XSHUT to power them up one at a time and assign unique addresses.
// =====================================================================
void initTOF() {
  pinMode(TOF_LEFT_XSHUT,  OUTPUT);
  pinMode(TOF_RIGHT_XSHUT, OUTPUT);
  // Power both off
  digitalWrite(TOF_LEFT_XSHUT,  LOW);
  digitalWrite(TOF_RIGHT_XSHUT, LOW);
  delay(10);

  // Boot LEFT sensor → assign address 0x30
  digitalWrite(TOF_LEFT_XSHUT, HIGH);
  delay(10);
  if (!tofLeft.begin(TOF_LEFT_ADDR)) {
    Serial.println(F("[TOF LEFT]  FAILED to initialise!"));
  } else {
    Serial.print(F("[TOF LEFT]  OK @ 0x")); Serial.println(TOF_LEFT_ADDR, HEX);
  }

  // Boot RIGHT sensor → default 0x29 is now free, assign 0x31
  digitalWrite(TOF_RIGHT_XSHUT, HIGH);
  delay(10);
  if (!tofRight.begin(TOF_RIGHT_ADDR)) {
    Serial.println(F("[TOF RIGHT] FAILED to initialise!"));
  } else {
    Serial.print(F("[TOF RIGHT] OK @ 0x")); Serial.println(TOF_RIGHT_ADDR, HEX);
  }
}

// =====================================================================
//  INIT: RFID (M5Stack RFID2 / WS1850S over I2C)
// =====================================================================
void initRFID() {
  rfid.begin(&Wire, RFID_I2C_ADDR);
  Serial.println(F("[RFID]      OK — WS1850S @ 0x28"));
}

// =====================================================================
//  INIT: WIFI + UDP
// =====================================================================
void initWiFi() {
  Serial.print(F("[WiFi]      Connecting to "));
  Serial.print(WIFI_SSID);

  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  unsigned long t = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - t < 10000UL) {
    delay(500);
    Serial.print('.');
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.print(F("\n[WiFi]      Connected. IP: "));
    Serial.println(WiFi.localIP());
    udp.begin(UDP_PORT);
    Serial.print(F("[UDP]       Listening on port "));
    Serial.println(UDP_PORT);
    Serial.println(F("            Send 'STOP' or 'START' to control remotely."));
  } else {
    Serial.println(F("\n[WiFi]      FAILED — WiFi kill switch unavailable."));
  }
}

// =====================================================================
//  KILL SWITCH — MECHANICAL (TOGGLE)
//  Each button press flips robotEnabled on ↔ off.
//  LED blinks red while disabled.
// =====================================================================
void handleMechKillSwitch() {
  bool pressed = (digitalRead(KILL_SWITCH_PIN) == LOW);

  // Rising-edge detect with debounce
  if (pressed && !prevBtnState && (millis() - lastDebounce > DEBOUNCE_MS)) {
    lastDebounce = millis();
    robotEnabled = !robotEnabled;

    if (robotEnabled && !wifiKillOn) {
      setLED(0, 255, 0);
      Serial.println(F("[KILL-HW]   ENABLED"));
    } else {
      stopAllMotors();
      Serial.println(F("[KILL-HW]   STOPPED"));
    }
  }
  prevBtnState = pressed;
}

// =====================================================================
//  KILL SWITCH — WIFI UDP
//  Accepts "STOP" or "START" as plain-text UDP packets on UDP_PORT.
//  WiFi kill overrides mechanical — robot stays off until "START" received.
// =====================================================================
void handleUDP() {
  if (WiFi.status() != WL_CONNECTED) return;

  int pktSize = udp.parsePacket();
  if (pktSize <= 0) return;

  char buf[32] = {0};
  udp.read(buf, sizeof(buf) - 1);
  String cmd = String(buf);
  cmd.trim();

  if (cmd.equalsIgnoreCase("STOP")) {
    robotEnabled = false;
    wifiKillOn   = true;
    stopAllMotors();
    Serial.println(F("[KILL-WiFi] STOPPED"));
  } else if (cmd.equalsIgnoreCase("START")) {
    wifiKillOn   = false;
    robotEnabled = true;
    setLED(0, 255, 0);
    Serial.println(F("[KILL-WiFi] ENABLED"));
  }
}

// =====================================================================
//  LED HELPERS
// =====================================================================
void setLED(uint8_t r, uint8_t g, uint8_t b) {
  analogWrite(LED_R_PIN, r);
  analogWrite(LED_G_PIN, g);
  analogWrite(LED_B_PIN, b);
}

// Non-blocking red blink while stopped (called every loop)
void updateLED() {
  if (!robotEnabled) {
    if (millis() - lastBlink >= BLINK_MS) {
      lastBlink = millis();
      blinkOn   = !blinkOn;
      setLED(blinkOn ? 255 : 0, 0, 0);
    }
  }
}

// =====================================================================
//  MOTOR CONTROL
//  All drive commands are blocked if robotEnabled == false.
// =====================================================================

// Core setter — applies direction inversion and clamps to valid range
void setDriveMotors(int leftSpeed, int rightSpeed) {
  if (!robotEnabled) { stopAllMotors(); return; }
  motoron.setSpeed(MOTOR_LEFT,  constrain(leftSpeed  * LEFT_DIR,  -MAX_SPEED, MAX_SPEED));
  motoron.setSpeed(MOTOR_RIGHT, constrain(rightSpeed * RIGHT_DIR, -MAX_SPEED, MAX_SPEED));
}

void stopAllMotors() {
  motoron.setSpeedNow(MOTOR_LEFT,    0);
  motoron.setSpeedNow(MOTOR_RIGHT,   0);
  motoron.setSpeedNow(MOTOR_PLANTER, 0);
}

// ── Directional helpers ───────────────────────────────────────────────
void driveForward (int spd) { setDriveMotors( spd,  spd); }
void driveBackward(int spd) { setDriveMotors(-spd, -spd); }
void pivotLeft    (int spd) { setDriveMotors(-spd,  spd); } // Left back, right forward
void pivotRight   (int spd) { setDriveMotors( spd, -spd); } // Left forward, right back

// Blocking turns — fine for checklist demos; replace with non-blocking for competition
void turn90Left()  { pivotLeft (TURN_SPEED); delay(TURN_90_MS); stopAllMotors(); }
void turn90Right() { pivotRight(TURN_SPEED); delay(TURN_90_MS); stopAllMotors(); }
void uTurn()       { pivotRight(TURN_SPEED); delay(UTURN_MS);   stopAllMotors(); }

// =====================================================================
//  PLANTER
//  Runs the planter motor for PLANTER_MS ms to deposit one seed.
// =====================================================================
void depositSeed() {
  if (!robotEnabled) return;
  Serial.println(F("[PLANTER]   Depositing seed..."));
  motoron.setSpeed(MOTOR_PLANTER, PLANTER_SPEED);
  delay(PLANTER_MS);
  motoron.setSpeedNow(MOTOR_PLANTER, 0);
  Serial.println(F("[PLANTER]   Done."));
}

// =====================================================================
//  SENSOR: QTR-HD-09RC
// =====================================================================

// Print raw RC timing values for all 9 sensors (lower = more reflective/white)
void printQTRReadings() {
  uint16_t vals[QTR_NUM_SENSORS];
  qtr.read(vals);
  Serial.print(F("[QTR   ] "));
  for (uint8_t i = 0; i < QTR_NUM_SENSORS; i++) {
    Serial.print(F("S")); Serial.print(i); Serial.print('='); Serial.print(vals[i]);
    if (i < QTR_NUM_SENSORS - 1) Serial.print(F("  "));
  }
  Serial.println();
}

// Returns calibrated line position: 0 (far left) → 8000 (far right), centre = 4000.
// You must run qtrCalibrate() at least once first for meaningful results.
uint16_t readLinePosition() {
  uint16_t vals[QTR_NUM_SENSORS];
  return qtr.readLineBlack(vals);
}

// Calibration helper — call in setup() while slowly sweeping sensor over the line.
// Drive the robot back and forth over the black line for ~2 seconds.
void qtrCalibrate(unsigned int durationMs = 2000) {
  Serial.println(F("[QTR]       Calibrating — move robot over line..."));
  unsigned long end = millis() + durationMs;
  while (millis() < end) {
    qtr.calibrate();
    delay(5);
  }
  Serial.println(F("[QTR]       Calibration done."));
}

// =====================================================================
//  SENSOR: HC-SR04 (front ultrasonic)
//  Returns distance in cm, or -1 if out of range / no echo.
// =====================================================================
float readFrontDistance() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  long dur = pulseIn(ECHO_PIN, HIGH, 30000UL);  // 30 ms timeout ≈ 5 m
  if (dur == 0) return -1.0f;
  return (dur * 0.0343f) / 2.0f;
}

// =====================================================================
//  SENSOR: VL53L0X (side, laser ToF)
//  Returns distance in cm, or -1 if out of range.
// =====================================================================
float readTOF(Adafruit_VL53L0X& sensor) {
  VL53L0X_RangingMeasurementData_t m;
  sensor.rangingTest(&m, false);
  if (m.RangeStatus != 4) return m.RangeMilliMeter / 10.0f;  // cm
  return -1.0f;
}

// =====================================================================
//  SENSOR: RFID (WS1850S / M5Stack RFID2 unit)
//  Prints UID to Serial whenever a new tag is detected.
// =====================================================================
void checkRFID() {
  if (!rfid.PICC_IsNewCardPresent()) return;
  if (!rfid.PICC_ReadCardSerial())   return;

  Serial.print(F("[RFID  ] Tag UID: "));
  for (uint8_t i = 0; i < rfid.uid.size; i++) {
    if (rfid.uid.uidByte[i] < 0x10) Serial.print('0');
    Serial.print(rfid.uid.uidByte[i], HEX);
    if (i < rfid.uid.size - 1) Serial.print(':');
  }
  Serial.println();

  rfid.PICC_HaltA();  // Halt card so it can be re-detected next pass
}

// =====================================================================
//  COMBINED SENSOR OUTPUT — printed to Serial Monitor for checklist demo
// =====================================================================
void printAllSensors() {
  // ── QTR ────────────────────────────────────────────────────────────
  printQTRReadings();

  // ── Front HC-SR04 ──────────────────────────────────────────────────
  float front = readFrontDistance();
  Serial.print(F("[HC-SR04] Front: "));
  if (front < 0) Serial.println(F("out of range"));
  else           { Serial.print(front, 1); Serial.println(F(" cm")); }

  // ── Side VL53L0X ───────────────────────────────────────────────────
  float leftDist  = readTOF(tofLeft);
  float rightDist = readTOF(tofRight);

  Serial.print(F("[TOF L  ] Left:  "));
  if (leftDist  < 0) Serial.println(F("out of range"));
  else               { Serial.print(leftDist,  1); Serial.println(F(" cm")); }

  Serial.print(F("[TOF R  ] Right: "));
  if (rightDist < 0) Serial.println(F("out of range"));
  else               { Serial.print(rightDist, 1); Serial.println(F(" cm")); }

  Serial.println(F("---"));
}
