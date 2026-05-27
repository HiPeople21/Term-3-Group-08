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

// --- Motor speed ---
static const int trackSpeed = 800;

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

// -----------------------------------------------------------------------

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000);

  // I2C for RFID + motors
  // Wire1.begin();

  // RFID reader
  // mfrc522.PCD_Init();
  Serial.println("[RFID] Reader ready.");

  // Motor shield (Wire1, address 0x12)
  // initMotors();
  Serial.println("[Motors] Ready.");

  // TOF sensors (Serial1 + Serial4) and ultrasonic (pins 44/42)
  // initSensors();
  Serial.println("[Sensors] TOF + Ultrasonic ready.");

  // IR array (QTR 12-sensor, ~10s calibration)
  // initIRArray();
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
  Serial.println("Tracks: W/S/A/D = Fwd/Rev/Left/Right | X = Stop");

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
    // stopTracks();
    // stopPlanter();
    wasPreviouslyKilled = true;
  } else if (!killed) {
    wasPreviouslyKilled = false;
  }
  // Serial.print(isKilledLocal);
  // Serial.print(" ");
  // Serial.println(isSystemEnabled());
  // Serial.println(killed);
  // Serial track control (disabled when killed)
  if (!killed && Serial.available() > 0) {
    char cmd = Serial.read();
    if      (cmd == 'w' || cmd == 'W') {Serial.println("Executed w"); setRightTrack(trackSpeed);  setLeftTrack(trackSpeed);  }
    else if (cmd == 's' || cmd == 'S') {Serial.println("Executed a"); setRightTrack(-trackSpeed); setLeftTrack(-trackSpeed); }
    else if (cmd == 'a' || cmd == 'A') {Serial.println("Executed s"); setRightTrack(trackSpeed);  setLeftTrack(-trackSpeed); }
    else if (cmd == 'd' || cmd == 'D') {Serial.println("Executed d"); setRightTrack(-trackSpeed); setLeftTrack(trackSpeed);  }
    else if (cmd == 'x' || cmd == 'X') {Serial.println("Executed x"); stopTracks(); }
    else if (cmd == '1')               { Serial.println("Executed 1"); openAirlockA(); }
    else if (cmd == '2')               { Serial.println("Executed 2"); openAirlockB(); }
    
    // Safely handling multi-part commands with parseInt
    else if (cmd == '3') { 
      Serial.println("Executed 3");
      int index = Serial.parseInt(); // Waits for and reads the next actual number
      // Serial.println(UIDs[index]);

      seedPlanted(UIDs[index]); 
    }
    else if (cmd == '4') { 
      Serial.println("Executed 4");
      int index = Serial.parseInt(); // Waits for and reads the next actual number
      // Serial.println(UIDs[index]);
      checkFertility(UIDs[index]); 
    }

  }

  // RFID — triggers planter rotation when card detected
  // checkRFID();

  // Drive planter motor toward target position (disabled when killed)
  // if (!killed) rotatePlanter();

  // Sensor readings — printed as fast as data arrives (TOF) or every 100ms (ultrasonic/IR)
  // readTOFSensors();
  // readUltrasonic();
  // readIRArray();
}
