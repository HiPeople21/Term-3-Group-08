#include "motors.h"
#include <Wire.h>
#include <Motoron.h>

static const float voltageScale = 6.0 / 7.2; // 6V motor rating / 7.2V battery

#define M1A 43
#define M1B 41
#define M2A 47
#define M2B 45
#define M3A 51
#define M3B 49

#define ENCODER_A M2A
#define ENCODER_B M2B

static MotoronI2C mc;

volatile long encoderPosPlanter = 0;
volatile long encoderPosTrack   = 0;
long targetPos = 0;

float countsPerRevolution = 1400.0;
long ticksFor60Degrees = countsPerRevolution / 6; // 233 ticks
int planterLinearspeed = 600;
int tracksDistance = 170; // 170 mm
float wheelDiameter = 38.5;
float wheelCircumference = wheelDiameter*PI;
float irToHole = 116.5;

float revolutionsFromIRToHole = irToHole / wheelDiameter * 1400;
// m1 = right track
// m2 = planter
// m3 = left track

void updateEncoder(int encoder_a, int encoder_b) {
  if (digitalRead(encoder_a) == digitalRead(encoder_b)) {
    encoderPosPlanter++;
  } else {
    encoderPosPlanter--;
  }
}

static void updateTrackEncoder() {
  if (digitalRead(M1A) == digitalRead(M1B)) {
    encoderPosTrack++;
  } else {
    encoderPosTrack--;
  }
}

void initMotors() {
  mc.setBus(&Wire1);
  mc.setAddress(18); // 0x12
  mc.reinitialize();
  mc.clearResetFlag();

  pinMode(ENCODER_A, INPUT_PULLUP);
  pinMode(ENCODER_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENCODER_A), [](){
    updateEncoder(ENCODER_A, ENCODER_B);
  }, CHANGE);

  pinMode(M1A, INPUT_PULLUP);
  pinMode(M1B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(M1A), updateTrackEncoder, CHANGE);
}

void setRightTrack(int speed) {
  mc.setSpeed(1, speed);
}

void setLeftTrack(int speed) {
  mc.setSpeed(3, speed);
}

void stopTracks() {
  mc.setSpeed(1, 0);
  mc.setSpeed(3, 0);
}

void setPlanter(int speed) {
  mc.setSpeed(2, speed);
}

void rotatePlanter() {
  long error = targetPos - encoderPosPlanter;
  int cmd = (error > 5) ? planterLinearspeed : 0;
  mc.setSpeed(2, cmd);
}

void stopPlanter() {
  mc.setSpeed(2, 0);
  targetPos = encoderPosPlanter;
}

void triggerPlanterRotation() {
  targetPos += ticksFor60Degrees;
}

long getTrackEncoder() {
  noInterrupts();
  long value = encoderPosTrack;
  interrupts();
  return value;
}

long getPlanterEncoder() {
  noInterrupts();
  long value = encoderPosPlanter;
  interrupts();
  return value;
}

static long turnStartTicks  = 0;
static long turnTicksNeeded = 0;
static int  turnDirSign     = 0;
static const int turnSpeed  = (int)(800 * voltageScale);

void startTurn(float degrees) {
  float arcLength = (tracksDistance / 2.0f) * abs(degrees) * PI / 180.0f * 4.75/4;
  turnTicksNeeded = (long)(arcLength / wheelCircumference * countsPerRevolution);

  noInterrupts();
  turnStartTicks = encoderPosTrack;
  interrupts();

  turnDirSign = (degrees > 0) ? 1 : -1;
  if (turnDirSign > 0) {
    setRightTrack(-turnSpeed);
    setLeftTrack(turnSpeed);
  } else {
    setRightTrack(turnSpeed);
    setLeftTrack(-turnSpeed);
  }
}

bool updateTurn() {
  noInterrupts();
  long pos = encoderPosTrack;
  interrupts();

  if (abs(pos - turnStartTicks) >= turnTicksNeeded) {
    stopTracks();
    return true;
  }
  return false;
}
