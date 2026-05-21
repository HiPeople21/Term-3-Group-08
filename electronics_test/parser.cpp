#include <map>
#include "parser.h"

std::map<String, String> parseToMap(String input) {
  std::map<String, String> dataMap;

  input.trim();

  int pairStart = 0;
  while (pairStart < (int)input.length()) {
    int pairEnd = input.indexOf(' ', pairStart);
    if (pairEnd == -1) pairEnd = input.length();

    String pair = input.substring(pairStart, pairEnd);
    pair.trim();

    if (pair.length() > 0) {
      int equalsIndex = pair.indexOf('=');
      if (equalsIndex != -1) {
        String key   = pair.substring(0, equalsIndex);
        String value = pair.substring(equalsIndex + 1);
        key.trim();
        value.trim();
        dataMap[key] = value;
      }
    }

    pairStart = pairEnd + 1;
  }

  return dataMap;
}
