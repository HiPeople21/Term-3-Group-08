#include <Wire.h>
#include "MFRC522_I2C.h"
#include "motors.h"

MFRC522_I2C mfrc522(0x28, -1, &Wire1);

enum RobotState {
  STATE_WAIT_RFID, STATE_STRAIGHT_1, STATE_TURN_RIGHT,
  STATE_STRAIGHT_2, STATE_TURN_LEFT, STATE_STRAIGHT_3, STATE_FINISHED
};
RobotState currentState = STATE_WAIT_RFID;

const int STRAIGHT_SPEED = 600;
int rfidCount = 0;


bool tagPresent = false;
unsigned long lastSeenMs = 0;
unsigned long lastCountMs = 0;
const unsigned long TAG_GONE_MS      = 500;  
const unsigned long COUNT_LOCKOUT_MS = 600;  

bool readTagOnce() {
  if (mfrc522.PICC_IsNewCardPresent() && mfrc522.PICC_ReadCardSerial()) {
    bool ok = (mfrc522.uid.size > 0);
    mfrc522.PICC_HaltA();
    return ok;
  }
  return false;
}

void armSegment() {            
  rfidCount = 0;
  tagPresent = true;           
  lastSeenMs = millis();
  lastCountMs = millis();      
}

void goStraight() {
  setRightTrack(STRAIGHT_SPEED);
  setLeftTrack(STRAIGHT_SPEED);
}


const float MY_TRACKS_DISTANCE     = 170.0;
const float MY_WHEEL_DIAMETER      = 38.5;
const float MY_WHEEL_CIRCUMFERENCE = MY_WHEEL_DIAMETER * PI;
const float MY_COUNTS_PER_REV      = 1400.0;

long turnStartTicks = 0, turnTicksNeeded = 0;
int  turnDirSign = 0;
const int turnSpeed = (int)(800 * 6 / 7.2);

void startTurnMain(float degrees) {
  float arcLength = (MY_TRACKS_DISTANCE / 2.0f) * abs(degrees) * PI / 180.0f * 4.15 / 4.0;
  turnTicksNeeded = (long)(arcLength / MY_WHEEL_CIRCUMFERENCE * MY_COUNTS_PER_REV);
  turnStartTicks  = getTrackEncoder();
  turnDirSign     = (degrees > 0) ? 1 : -1;

  digitalWrite(LED_BUILTIN, HIGH);   
  Serial.print("[TURN] startTicks="); Serial.print(turnStartTicks);
  Serial.print("  need=");            Serial.println(turnTicksNeeded);

  if (turnDirSign > 0) { setRightTrack(-turnSpeed); setLeftTrack(turnSpeed); }
  else                 { setRightTrack(turnSpeed);  setLeftTrack(-turnSpeed); }
}

bool updateTurnMain() {
  long pos = getTrackEncoder();
  /
  Serial.print("[TURN] pos="); Serial.println(pos);
  if (abs(pos - turnStartTicks) >= turnTicksNeeded) {
    stopTracks();
    digitalWrite(LED_BUILTIN, LOW);
    return true;
  }
  return false;
}

void setup() {
  Serial.begin(9600);
  Wire1.begin();
  initMotors();
  mfrc522.PCD_Init();
  pinMode(LED_BUILTIN, OUTPUT);
  for (int i = 0; i < 5; i++) {      /
    digitalWrite(LED_BUILTIN, HIGH); delay(100);
    digitalWrite(LED_BUILTIN, LOW);  delay(100);
  }
  Serial.println("Ready. Place robot on 1st tag.");
  Serial.print("Encoder at boot = "); Serial.println(getTrackEncoder());
}

void loop() {
  bool hasNewTag = false;
  bool seen = readTagOnce();
  unsigned long now = millis();

  if (seen) {
    lastSeenMs = now;
    if (!tagPresent) {                      
      tagPresent = true;
      if (now - lastCountMs > COUNT_LOCKOUT_MS) {  /
        hasNewTag = true;
        lastCountMs = now;
      }
    }
  } else {
    if (tagPresent && (now - lastSeenMs > TAG_GONE_MS)) tagPresent = false;
  }

  switch (currentState) {
    case STATE_WAIT_RFID:
      if (hasNewTag) {
        delay(3000);
        Serial.println("[START] -> STRAIGHT_1");
        armSegment(); currentState = STATE_STRAIGHT_1; goStraight();
      }
      break;

    case STATE_STRAIGHT_1:
      if (hasNewTag) {
        rfidCount++;
        Serial.print("[S1] count="); Serial.println(rfidCount);
        if (rfidCount == 2) { stopTracks(); delay(200); startTurnMain(90.0); currentState = STATE_TURN_RIGHT; }
      }
      break;

    case STATE_TURN_RIGHT:
      if (updateTurnMain()) { delay(200); armSegment(); currentState = STATE_STRAIGHT_2; goStraight(); }
      break;

    case STATE_STRAIGHT_2:
      if (hasNewTag) {
        rfidCount++;
        Serial.print("[S2] count="); Serial.println(rfidCount);
        if (rfidCount == 1) { stopTracks(); delay(200); startTurnMain(-98.0); currentState = STATE_TURN_LEFT; }
      }
      break;

    case STATE_TURN_LEFT:
      if (updateTurnMain()) { delay(200); armSegment(); currentState = STATE_STRAIGHT_3; goStraight(); }
      break;

    case STATE_STRAIGHT_3:
      if (hasNewTag) {
        rfidCount++;
        Serial.print("[S3] count="); Serial.println(rfidCount);
        if (rfidCount == 2) { stopTracks(); Serial.println("[FINISHED]"); currentState = STATE_FINISHED; }
      }
      break;

    case STATE_FINISHED:
      break;
  }

   switch (currentState) {
    case STATE_STRAIGHT_1:
    case STATE_STRAIGHT_2:
    case STATE_STRAIGHT_3:
      setRightTrack(STRAIGHT_SPEED);
      setLeftTrack(STRAIGHT_SPEED);
      break;
    case STATE_TURN_RIGHT:
      setRightTrack(-turnSpeed); setLeftTrack(turnSpeed);
      break;
    case STATE_TURN_LEFT:
      setRightTrack(turnSpeed);  setLeftTrack(-turnSpeed);
      break;
    default: break;
  }
}
