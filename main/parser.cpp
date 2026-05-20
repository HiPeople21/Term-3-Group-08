#include <map>
#include "parser.h"

// --- String to Hashmap Parser (Space Separated) ---
std::map<String, String> parseToMap(String input) {
  std::map<String, String> dataMap;
  
  input.trim(); // Clean up the edges

  int pairStart = 0;
  
  // Loop through the string, splitting by space instead of comma
  while (pairStart < input.length()) {
    int pairEnd = input.indexOf(' ', pairStart);
    
    // If there are no more spaces, this is the last pair
    if (pairEnd == -1) {
      pairEnd = input.length(); 
    }

    // Extract the "key=value" chunk
    String pair = input.substring(pairStart, pairEnd);
    pair.trim();

    // Only process if the chunk isn't empty (safeguard against double-spaces)
    if (pair.length() > 0) {
      // Split the chunk by the '=' sign
      int equalsIndex = pair.indexOf('=');
      
      if (equalsIndex != -1) {
        String key = pair.substring(0, equalsIndex);
        String value = pair.substring(equalsIndex + 1);

        key.trim();
        value.trim();

        // Add it to our hashmap!
        dataMap[key] = value;
      }
    }

    // Move to the next chunk (skip the space)
    pairStart = pairEnd + 1;
  }

  return dataMap;
}