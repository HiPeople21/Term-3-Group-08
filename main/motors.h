#pragma once
#include <Arduino.h>

void initMotors();
void setRightTrack(int speed);
void setLeftTrack(int speed);
void stopTracks();
void stopPlanter();
void rotatePlanter();
void triggerPlanterRotation();
