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

// Point-turn helpers — call startTurn() once, then updateTurn() every loop.
// Positive degrees = right, negative = left.
void startTurn(float degrees);
bool updateTurn();
