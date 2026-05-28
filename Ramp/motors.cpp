#include "motors.h"
#include <Wire.h>
#include <Motoron.h>

static MotoronI2C mc;

void initMotors() {
  mc.setBus(&Wire1);
  mc.setAddress(18); // 0x12
  mc.reinitialize();
  mc.clearResetFlag();
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
