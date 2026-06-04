#pragma once

#include <Arduino.h>
#include <map>

// --- String to Hashmap Parser ---
std::map<String, String> parseToMap(String input);