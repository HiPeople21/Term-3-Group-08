#pragma once
#include <Arduino.h>
#include <Motoron.h>


void initMotors();
void setRightTrack(int speed);
void setLeftTrack(int speed);
void setPlanter(int speed);

void stopTracks();
