// =====================================================================
//  UCL RAI Robotics Challenge 2026 — Team 19
//  Electronics Checklist Demo  (Trial Run #1)
//
//  Single program cycling through ALL electronics checklist items:
//    [1] Mechanical kill switch   → toggle stop/run, blink red when stopped
//    [2] WiFi UDP kill switch     → STOP / START / EMERGENCY on port 4210
//    [3] Speed & heading control  → slow, fast, turn L/R 90°, U-turn
//    [4] Reflectance sensors      → IR centre(9) + left(2) + right(2) on Serial
//    [5] Distance sensors         → HC-SR04 ultrasonic + VL53L0X ToF on Serial
//    [6] RFID                     → tag UID printed on Serial when detected
//
//  LED states:
//    Solid BLUE   → robot running (demo cycling)
//    Blinking RED → robot stopped by kill switch
//    Solid GREEN  → revive button held (overrides everything)
//
//  Serial baud : 115200
// =====================================================================

// ─── Libraries ───────────────────────────────────────────────────────
#include <Wire.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <Motoron.h>
#include <MFRC522_I2C.h>
#include <QTRSensors.h>
#include <VL53L0X.h>

// ─────────────────────────────────────────────────────────────────────
//  NETWORK CONFIG  ← set your lab WiFi credentials here
// ─────────────────────────────────────────────────────────────────────
const char* WIFI_SSID = "YOUR_SSID";
const char* WIFI_PASS = "YOUR_PASSWORD";
const int   UDP_PORT  = 4210;

// ─────────────────────────────────────────────────────────────────────
//  PIN DEFINITIONS
// ─────────────────────────────────────────────────────────────────────

// Kill switch & revive buttons (INPUT_PULLUP → active LOW)
const int PIN_KILL          = 2;
const int PIN_REVIVE_LEFT   = 3;
const int PIN_REVIVE_RIGHT  = 4;

// RGB LED — common cathode: HIGH = ON
//   Physical pinout on board: R – GND – G – B
const int PIN_LED_R = 9;
const int PIN_LED_G = 10;
const int PIN_LED_B = 11;

// HC-SR04 ultrasonic (ECHO reassigned to 12 — pin 6 physically blocked)
const int PIN_TRIG = 5;
const int PIN_ECHO = 12;

// VL53L0X XSHUT pins (pull LOW to disable, HIGH to enable)
const int PIN_TOF_L_XSHUT = 48;
const int PIN_TOF_R_XSHUT = 50;

// QTR-HD-09RC — centre 9-sensor array
const uint8_t IR_CENTRE_PINS[9] = {22, 24, 26, 28, 30, 32, 34, 36, 38};

// QTR-HD-02RC — left & right 2-sensor arrays
const uint8_t IR_LEFT_PINS[2]  = {40, 42};
const uint8_t IR_RIGHT_PINS[2] = {44, 46};

// ─────────────────────────────────────────────────────────────────────
//  HARDWARE OBJECTS
// ─────────────────────────────────────────────────────────────────────

WiFiUDP        udp;
MotoronI2C     mc;                         // Motoron M3S550 on Wire1 @ 0x12
MFRC522_I2C    rfid(0x28, -1, &Wire1);     // WS1850S RFID on Wire1 @ 0x28
VL53L0X        tofLeft, tofRight;          // VL53L0X ToF on Wire  @ 0x30, 0x31

QTRSensors     qtrCentre, qtrLeft, qtrRight;
uint16_t       irCentre[9];
uint16_t       irLeft[2];
uint16_t       irRight[2];

// ─────────────────────────────────────────────────────────────────────
//  MOTOR HELPERS
// ─────────────────────────────────────────────────────────────────────
//
//  Motor 1 → Right track   (Motoron channel 1)
//  Motor 3 → Left  track   (Motoron channel 3)
//  Positive speed = forward for both tracks.
//
//  ┌──── TUNING NOTE ────────────────────────────────────────────────┐
//  │ TURN_90_MS  : time (ms) for a ~90° pivot at SPEED_TURN=400     │
//  │ TURN_180_MS : should be ≈ 2 × TURN_90_MS                       │
//  │ Increase if robot under-rotates; decrease if it over-rotates.  │
//  └─────────────────────────────────────────────────────────────────┘
const unsigned long TURN_90_MS  = 600;
const unsigned long TURN_180_MS = 1200;

void setMotors(int leftSpeed, int rightSpeed) {
  mc.setSpeed(1, rightSpeed);
  mc.setSpeed(3, leftSpeed);
}

void stopMotors() {
  mc.setSpeed(1, 0);
  mc.setSpeed(3, 0);
}

// ─────────────────────────────────────────────────────────────────────
//  KILL SWITCH & REVIVE STATE
// ─────────────────────────────────────────────────────────────────────

bool isStopped = false;

// Kill switch debounce
int           killLastRead    = HIGH;
int           killStableState = HIGH;
unsigned long killDebounceTime = 0;

// Revive button debounce
int  revLLastRead = HIGH, revLStable = HIGH; unsigned long revLDebounce = 0;
int  revRLastRead = HIGH, revRStable = HIGH; unsigned long revRDebounce = 0;
bool wasRevived   = false;

const unsigned long DEBOUNCE_MS = 50;

// Blink state for stopped indicator
unsigned long blinkPrev = 0;
const long    BLINK_MS  = 500;
bool          blinkOn   = false;

// ─────────────────────────────────────────────────────────────────────
//  DEMO SEQUENCE STATE MACHINE
// ─────────────────────────────────────────────────────────────────────
//
//  Steps cycle continuously while the robot is running.
//  Kill switch pauses the cycle mid-step; resuming continues from
//  the beginning of the same step.

enum DemoStep {
  DEMO_FWD_SLOW,    // Forward at speed 200 for 2 s
  DEMO_FWD_FAST,    // Forward at speed 600 for 2 s
  DEMO_PAUSE_1,     // Stop for 0.5 s
  DEMO_TURN_LEFT,   // Pivot left  ~90°
  DEMO_TURN_RIGHT,  // Pivot right ~90°
  DEMO_UTURN,       // Pivot left  ~180° (U-turn)
  DEMO_PAUSE_2      // Stop for 0.5 s, then restart cycle
};

DemoStep      demoStep   = DEMO_FWD_SLOW;
unsigned long stepStart  = 0;
bool          stepActive = false;  // true once motors commanded for this step

// Durations
const unsigned long STEP_SLOW_MS  = 2000;
const unsigned long STEP_FAST_MS  = 2000;
const unsigned long STEP_PAUSE_MS = 500;

// Speed values (Motoron range: -800 to 800)
const int SPEED_SLOW = 200;
const int SPEED_FAST = 600;
const int SPEED_TURN = 400;

// ─────────────────────────────────────────────────────────────────────
//  SENSOR PRINT TIMER
// ─────────────────────────────────────────────────────────────────────

unsigned long sensorPrintPrev = 0;
const unsigned long SENSOR_PRINT_MS = 200;

// ─────────────────────────────────────────────────────────────────────
//  LED HELPERS
// ─────────────────────────────────────────────────────────────────────

void ledOff()        { digitalWrite(PIN_LED_R, LOW);  digitalWrite(PIN_LED_G, LOW);  digitalWrite(PIN_LED_B, LOW);  }
void ledSolidBlue()  { digitalWrite(PIN_LED_R, LOW);  digitalWrite(PIN_LED_G, LOW);  digitalWrite(PIN_LED_B, HIGH); }
void ledSolidGreen() { digitalWrite(PIN_LED_R, LOW);  digitalWrite(PIN_LED_G, HIGH); digitalWrite(PIN_LED_B, LOW);  }
void ledSolidRed()   { digitalWrite(PIN_LED_R, HIGH); digitalWrite(PIN_LED_G, LOW);  digitalWrite(PIN_LED_B, LOW);  }

// ─────────────────────────────────────────────────────────────────────
//  ULTRASONIC HELPER
// ─────────────────────────────────────────────────────────────────────

float readUltrasonic() {
  digitalWrite(PIN_TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(PIN_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(PIN_TRIG, LOW);
  long dur = pulseIn(PIN_ECHO, HIGH, 30000UL);  // 30 ms timeout ≈ 5 m max
  return (dur == 0) ? -1.0f : dur / 58.0f;
}

// ─────────────────────────────────────────────────────────────────────
//  SETUP
// ─────────────────────────────────────────────────────────────────────

void setup() {

  Serial.begin(115200);
  while (!Serial);

  // ── GPIO ──────────────────────────────────────────────────────────
  pinMode(PIN_KILL,         INPUT_PULLUP);
  pinMode(PIN_REVIVE_LEFT,  INPUT_PULLUP);
  pinMode(PIN_REVIVE_RIGHT, INPUT_PULLUP);
  pinMode(PIN_LED_R, OUTPUT);
  pinMode(PIN_LED_G, OUTPUT);
  pinMode(PIN_LED_B, OUTPUT);
  pinMode(PIN_TRIG,  OUTPUT);
  pinMode(PIN_ECHO,  INPUT);

  ledSolidBlue();  // boot state: running

  // ── I2C buses ─────────────────────────────────────────────────────
  Wire.begin();    // default bus  → VL53L0X sensors
  Wire1.begin();   // secondary    → Motoron + RFID

  // ── Motoron M3S550 ────────────────────────────────────────────────
  mc.setBus(&Wire1);
  mc.setAddress(18);  // hex 0x12
  mc.reinitialize();
  mc.clearResetFlag();
  stopMotors();
  Serial.println("[MOTORON] OK");

  // ── RFID (WS1850S / MFRC522-compatible, I2C @ 0x28) ──────────────
  rfid.PCD_Init();
  Serial.println("[RFID] OK");

  // ── VL53L0X — boot one sensor at a time using XSHUT ──────────────
  pinMode(PIN_TOF_L_XSHUT, OUTPUT);
  pinMode(PIN_TOF_R_XSHUT, OUTPUT);
  digitalWrite(PIN_TOF_L_XSHUT, LOW);   // both off initially
  digitalWrite(PIN_TOF_R_XSHUT, LOW);
  delay(10);

  // Left sensor → assign address 0x30
  digitalWrite(PIN_TOF_L_XSHUT, HIGH);
  delay(10);
  tofLeft.setBus(&Wire);
  tofLeft.setAddress(0x30);
  if (!tofLeft.init()) Serial.println("[TOF LEFT] INIT FAILED — check wiring");
  else { tofLeft.startContinuous(); Serial.println("[TOF LEFT]  OK @ 0x30"); }

  // Right sensor → assign address 0x31
  digitalWrite(PIN_TOF_R_XSHUT, HIGH);
  delay(10);
  tofRight.setBus(&Wire);
  tofRight.setAddress(0x31);
  if (!tofRight.init()) Serial.println("[TOF RIGHT] INIT FAILED — check wiring");
  else { tofRight.startContinuous(); Serial.println("[TOF RIGHT] OK @ 0x31"); }

  // ── QTR IR sensor arrays ──────────────────────────────────────────
  qtrCentre.setTypeRC();
  qtrCentre.setSensorPins(IR_CENTRE_PINS, 9);

  qtrLeft.setTypeRC();
  qtrLeft.setSensorPins(IR_LEFT_PINS, 2);

  qtrRight.setTypeRC();
  qtrRight.setSensorPins(IR_RIGHT_PINS, 2);

  Serial.println("[QTR]    OK  (9-centre, 2-left, 2-right)");

  // ── WiFi + UDP kill switch ─────────────────────────────────────────
  Serial.print("[WiFi]   Connecting to ");
  Serial.print(WIFI_SSID);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  int tries = 0;
  while (WiFi.status() != WL_CONNECTED && tries < 20) {
    delay(500); Serial.print("."); tries++;
  }
  if (WiFi.status() == WL_CONNECTED) {
    udp.begin(UDP_PORT);
    Serial.print("\n[WiFi]   Connected. IP: ");
    Serial.println(WiFi.localIP());
    Serial.print("[UDP]    Listening on port ");
    Serial.println(UDP_PORT);
  } else {
    Serial.println("\n[WiFi]   NOT connected — UDP kill switch unavailable");
  }

  stepStart = millis();

  Serial.println();
  Serial.println("================================================");
  Serial.println(" ELECTRONICS CHECKLIST DEMO — RUNNING");
  Serial.println(" Solid BLUE = running | Blink RED = stopped");
  Serial.println(" Solid GREEN = revive button held");
  Serial.println("================================================");
  Serial.println("[IR-C] c0,c1,c2,c3,c4,c5,c6,c7,c8  [IR-L] l0,l1  [IR-R] r0,r1  [US] cm  [TOF-L] mm  [TOF-R] mm");
}

// ─────────────────────────────────────────────────────────────────────
//  MAIN LOOP
// ─────────────────────────────────────────────────────────────────────

void loop() {

  unsigned long now = millis();

  // ================================================================
  //  [1] MECHANICAL KILL SWITCH — debounced toggle
  // ================================================================
  {
    int r = digitalRead(PIN_KILL);
    if (r != killLastRead) killDebounceTime = now;
    if ((now - killDebounceTime) > DEBOUNCE_MS && r != killStableState) {
      killStableState = r;
      if (killStableState == LOW) {          // falling edge = button pressed
        isStopped = !isStopped;
        if (isStopped) {
          stopMotors();
          stepActive = false;
          blinkPrev = now; blinkOn = false;
          ledSolidRed();
          Serial.println("[KILL SW] → STOPPED  (blinking RED)");
        } else {
          stepStart = now;                   // resume from start of current step
          ledSolidBlue();
          Serial.println("[KILL SW] → RUNNING  (solid BLUE)");
        }
      }
    }
    killLastRead = r;
  }

  // ================================================================
  //  [2] WiFi UDP KILL SWITCH
  //      Send UDP packets to robot's IP, port 4210:
  //        "STOP"      → stop robot
  //        "START"     → resume robot
  //        "EMERGENCY" → stop robot (same as STOP for demo)
  // ================================================================
  if (WiFi.status() == WL_CONNECTED) {
    int pkt = udp.parsePacket();
    if (pkt > 0) {
      char buf[32] = {0};
      udp.read(buf, sizeof(buf) - 1);
      String msg = String(buf);
      msg.trim();

      if (msg == "STOP" && !isStopped) {
        isStopped = true;
        stopMotors();
        stepActive = false;
        blinkPrev = now; blinkOn = false;
        ledSolidRed();
        Serial.println("[UDP]    STOP  → STOPPED  (blinking RED)");

      } else if ((msg == "START") && isStopped) {
        isStopped = false;
        stepStart = now;
        ledSolidBlue();
        Serial.println("[UDP]    START → RUNNING  (solid BLUE)");

      } else if (msg == "EMERGENCY") {
        isStopped = true;
        stopMotors();
        stepActive = false;
        ledSolidRed();
        Serial.println("[UDP]    EMERGENCY → STOPPED");
      }
    }
  }

  // ================================================================
  //  REVIVE BUTTONS — green LED override while either button held
  // ================================================================
  {
    int rL = digitalRead(PIN_REVIVE_LEFT);
    if (rL != revLLastRead) revLDebounce = now;
    if ((now - revLDebounce) > DEBOUNCE_MS) revLStable = rL;
    revLLastRead = rL;

    int rR = digitalRead(PIN_REVIVE_RIGHT);
    if (rR != revRLastRead) revRDebounce = now;
    if ((now - revRDebounce) > DEBOUNCE_MS) revRStable = rR;
    revRLastRead = rR;
  }

  bool reviveHeld = (revLStable == LOW || revRStable == LOW);

  if (reviveHeld && !wasRevived) {
    wasRevived = true;
    ledSolidGreen();
    Serial.println("[REVIVE] Button held   → solid GREEN");
  } else if (!reviveHeld && wasRevived) {
    wasRevived = false;
    isStopped ? ledSolidRed() : ledSolidBlue();
    Serial.println("[REVIVE] Button released");
  }

  // ================================================================
  //  BLINK RED while stopped (and revive not held)
  // ================================================================
  if (isStopped && !reviveHeld) {
    if (now - blinkPrev >= (unsigned long)BLINK_MS) {
      blinkPrev = now;
      blinkOn = !blinkOn;
      blinkOn ? ledSolidRed() : ledOff();
    }
  }

  // ================================================================
  //  [3] DEMO SEQUENCE — speed & heading  (only while running)
  // ================================================================
  if (!isStopped) {
    runDemoStep(now);
  }

  // ================================================================
  //  [4] + [5] SENSOR PRINT — IR, ultrasonic, ToF  (every 200 ms)
  // ================================================================
  if (now - sensorPrintPrev >= SENSOR_PRINT_MS) {
    sensorPrintPrev = now;
    printSensors();
  }

  // ================================================================
  //  [6] RFID — non-blocking check every loop iteration
  // ================================================================
  if (rfid.PICC_IsNewCardPresent() && rfid.PICC_ReadCardSerial()) {
    Serial.print("[RFID]   Tag UID: ");
    for (byte i = 0; i < rfid.uid.size; i++) {
      if (rfid.uid.uidByte[i] < 0x10) Serial.print("0");
      Serial.print(rfid.uid.uidByte[i], HEX);
      if (i < rfid.uid.size - 1) Serial.print(":");
    }
    Serial.println();
    rfid.PICC_HaltA();
  }
}

// ─────────────────────────────────────────────────────────────────────
//  DEMO STEP RUNNER
// ─────────────────────────────────────────────────────────────────────

void runDemoStep(unsigned long now) {

  unsigned long elapsed = now - stepStart;

  switch (demoStep) {

    // ── Step 1: Forward SLOW ──────────────────────────────────────
    case DEMO_FWD_SLOW:
      if (!stepActive) {
        setMotors(SPEED_SLOW, SPEED_SLOW);
        Serial.print("[DEMO] Forward SLOW  speed=");
        Serial.println(SPEED_SLOW);
        stepActive = true;
      }
      if (elapsed >= STEP_SLOW_MS) advanceStep(now);
      break;

    // ── Step 2: Forward FAST ──────────────────────────────────────
    case DEMO_FWD_FAST:
      if (!stepActive) {
        setMotors(SPEED_FAST, SPEED_FAST);
        Serial.print("[DEMO] Forward FAST  speed=");
        Serial.println(SPEED_FAST);
        stepActive = true;
      }
      if (elapsed >= STEP_FAST_MS) advanceStep(now);
      break;

    // ── Step 3: Pause ─────────────────────────────────────────────
    case DEMO_PAUSE_1:
      if (!stepActive) {
        stopMotors();
        Serial.println("[DEMO] Pause");
        stepActive = true;
      }
      if (elapsed >= STEP_PAUSE_MS) advanceStep(now);
      break;

    // ── Step 4: Turn LEFT ~90° ────────────────────────────────────
    //   Left track backward, right track forward → pivot left
    case DEMO_TURN_LEFT:
      if (!stepActive) {
        setMotors(-SPEED_TURN, SPEED_TURN);
        Serial.println("[DEMO] Turn LEFT  ~90°");
        stepActive = true;
      }
      if (elapsed >= TURN_90_MS) advanceStep(now);
      break;

    // ── Step 5: Turn RIGHT ~90° ───────────────────────────────────
    //   Left track forward, right track backward → pivot right
    case DEMO_TURN_RIGHT:
      if (!stepActive) {
        setMotors(SPEED_TURN, -SPEED_TURN);
        Serial.println("[DEMO] Turn RIGHT ~90°");
        stepActive = true;
      }
      if (elapsed >= TURN_90_MS) advanceStep(now);
      break;

    // ── Step 6: U-turn (~180°) ────────────────────────────────────
    //   Same direction as left turn, for twice as long
    case DEMO_UTURN:
      if (!stepActive) {
        setMotors(-SPEED_TURN, SPEED_TURN);
        Serial.println("[DEMO] U-turn    ~180°");
        stepActive = true;
      }
      if (elapsed >= TURN_180_MS) advanceStep(now);
      break;

    // ── Step 7: End-of-cycle pause, then repeat ───────────────────
    case DEMO_PAUSE_2:
      if (!stepActive) {
        stopMotors();
        Serial.println("[DEMO] Cycle complete — restarting sequence");
        stepActive = true;
      }
      if (elapsed >= STEP_PAUSE_MS) {
        demoStep   = DEMO_FWD_SLOW;
        stepStart  = now;
        stepActive = false;
      }
      break;
  }
}

// Advance to the next demo step
void advanceStep(unsigned long now) {
  demoStep   = (DemoStep)((int)demoStep + 1);
  stepStart  = now;
  stepActive = false;
}

// ─────────────────────────────────────────────────────────────────────
//  SENSOR PRINT  (one compact line every 200 ms)
// ─────────────────────────────────────────────────────────────────────

void printSensors() {

  // ── IR Centre array (9 sensors) ──────────────────────────────────
  qtrCentre.read(irCentre);
  Serial.print("[IR-C] ");
  for (int i = 0; i < 9; i++) {
    Serial.print(irCentre[i]);
    if (i < 8) Serial.print(",");
  }

  // ── IR Left array (2 sensors) ────────────────────────────────────
  qtrLeft.read(irLeft);
  Serial.print("  [IR-L] ");
  Serial.print(irLeft[0]); Serial.print(","); Serial.print(irLeft[1]);

  // ── IR Right array (2 sensors) ───────────────────────────────────
  qtrRight.read(irRight);
  Serial.print("  [IR-R] ");
  Serial.print(irRight[0]); Serial.print(","); Serial.print(irRight[1]);

  // ── HC-SR04 Ultrasonic ───────────────────────────────────────────
  float usDist = readUltrasonic();
  Serial.print("  [US] ");
  if (usDist < 0) Serial.print("OOR");
  else { Serial.print(usDist, 1); Serial.print("cm"); }

  // ── VL53L0X ToF (left + right) ───────────────────────────────────
  uint16_t dL = tofLeft.readRangeContinuousMillimeters();
  uint16_t dR = tofRight.readRangeContinuousMillimeters();
  Serial.print("  [TOF-L] "); Serial.print(dL); Serial.print("mm");
  Serial.print("  [TOF-R] "); Serial.print(dR); Serial.print("mm");

  Serial.println();
}
