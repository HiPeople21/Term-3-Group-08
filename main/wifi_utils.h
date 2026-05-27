#pragma once
#include <Arduino.h>

extern String UIDs[];

void initWifi();
void loopWifi();
bool isSystemEnabled();
void setupGrid();
void checkFertility(String tagId);
void seedPlanted(String tagId);
void openAirlockA();
void openAirlockB();
void register_bot();