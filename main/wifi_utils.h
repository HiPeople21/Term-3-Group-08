#pragma once
#include <Arduino.h>

extern String UIDs[];

void initWifi();
void loopWifi();
bool isSystemEnabled();
void setupGrid();
void checkFertility(String tagId);
void seedPlanted(String tagId);
void openAirlock(String tagId, char airlock);
void register_bot();
void setFertilityCallback(void (*cb)(bool fertile));
void reviveRequest(int target_team, String target_board);
void getMap();
