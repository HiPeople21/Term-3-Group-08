#pragma once
#include <Arduino.h>

void initMotors();
void setRightTrack(int speed);
void setLeftTrack(int speed);
void stopTracks();
void stopPlanter();
void setPlanter(int speed);
void rotatePlanter();
void triggerPlanterRotation();
long getTrackEncoder();
long getPlanterEncoder();
