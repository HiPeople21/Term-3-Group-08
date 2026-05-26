#include "motors.h"
#include <Wire.h>
#include <Motoron.h>

#define M1A 41
#define M1B 43
#define M2A 47
#define M2B 45
#define M3A 49
#define M3B 51

#define ENCODER_A M2A
#define ENCODER_B M2B

static MotoronI2C mc;

volatile long encoderPosPlanter = 0;
long targetPos = 0;

float countsPerRevolution = 1400.0;
long ticksFor60Degrees = countsPerRevolution / 6; // 250 ticks
int planterLinearspeed = 600;
// m1 = right track
// m2 = planter
// m3 = left track

void updateEncoder() {
  if (digitalRead(ENCODER_A) == digitalRead(ENCODER_B)) {
    encoderPosPlanter++;
  } else {
    encoderPosPlanter--;
  }
}

void initMotors() {
  mc.setBus(&Wire1);
  mc.setAddress(18); // 0x12
  mc.reinitialize();
  mc.clearResetFlag();

  pinMode(ENCODER_A, INPUT_PULLUP);
  pinMode(ENCODER_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENCODER_A), updateEncoder, CHANGE);
}

void setRightTrack(int speed) {
  mc.setSpeed(1, -speed);
}

void setLeftTrack(int speed) {
  mc.setSpeed(3, speed);
}

void stopTracks() {
  mc.setSpeed(1, 0);
  mc.setSpeed(3, 0);
}

void rotatePlanter() {
  long error = targetPos - encoderPosPlanter;
  int cmd = (error > 5) ? planterLinearspeed : 0;
  mc.setSpeed(2, cmd);

  static unsigned long lastPrint = 0;
  if (millis() - lastPrint > 500) {
    lastPrint = millis();
    Serial.print("[Planter] encoder="); Serial.print(encoderPosPlanter);
    Serial.print(" target="); Serial.print(targetPos);
    Serial.print(" error="); Serial.println(error);
  }
}

void stopPlanter() {
  mc.setSpeed(2, 0);
  targetPos = encoderPosPlanter; // cancel any pending rotation
}

void triggerPlanterRotation() {
  targetPos += ticksFor60Degrees;
}