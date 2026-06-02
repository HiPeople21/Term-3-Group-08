#pragma once
#include <Arduino.h>

void initSensors();
void readTOFSensors();
void readUltrasonic();

void initIRArray();
void readIRArray();

uint16_t readIRPosition();
uint16_t getIRValue(uint8_t index);
uint8_t  getIRSensorCount();
