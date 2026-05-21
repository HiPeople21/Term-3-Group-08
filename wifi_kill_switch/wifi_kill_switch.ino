#include <MiniMessenger.h>
#include "parser.h"
#include "secrets.h"

// --- Standard External LED Configuration ---
#define LED_ON  HIGH
#define LED_OFF LOW

// --- State Variables ---
const int redPin = 38;        // Connect to the Red leg of the RGB LED
const int greenPin = 40;      // Connect to the Green leg of the RGB LED
unsigned long lastRegisterMs = 0;
bool isBlinkingRed = false;   // false = Solid Green, true = Blinking Red

// --- Blinking Variables ---
unsigned long previousMillis = 0;    
const long blinkInterval = 500;      
int redLedState = LED_OFF;              

MiniMessenger messenger;
const char* BoardId = "bot";  

// --- Helper Functions ---

void setLedSolidGreen() {
  digitalWrite(redPin, LED_OFF); // Turn Red OFF
  digitalWrite(greenPin, LED_ON);  // Turn Green ON
}

void toggleSystemState() {
  isBlinkingRed = !isBlinkingRed; 

  if (!isBlinkingRed) {
    setLedSolidGreen(); 
  } else {
    // Transitioning to Red: Turn Green OFF immediately
    digitalWrite(greenPin, LED_OFF); // <--- FIXED: Now turns off the green pin
    
    // Turn Red ON immediately so the blink starts right away
    redLedState = LED_ON; 
    digitalWrite(redPin, redLedState);
    previousMillis = millis(); 
  }
}

// --- Messenger Callback ---

void onMessage(const MessageMetadata& metadata, const uint8_t* payload, size_t length) {
  
  // 1. Build the string
  String incomingMsg = "";
  for (size_t i = 0; i < length; i++) {
    incomingMsg += (char)payload[i];
  }
  incomingMsg.trim(); 

  // 2. FILTER THE PING: If the message is completely empty, silently ignore it
  if (incomingMsg.length() == 0) return; 

  std::map<String, String> commandMap = parseToMap(incomingMsg);
  
  // Print it!
  Serial.print("Message from Board ");
  Serial.print(metadata.fromBoardId);
  Serial.print(": ");
  Serial.println(incomingMsg);

  // 3. COMMAND PROCESSOR — only act on type=disable reason=operator enabled=<bool>
  bool isDisableCmd = commandMap.count("type")   && commandMap["type"].equalsIgnoreCase("disable");
  bool isOperator   = commandMap.count("reason") && commandMap["reason"].equalsIgnoreCase("operator");
  bool hasEnabled   = commandMap.count("enabled") > 0;

  if (!isDisableCmd || !isOperator || !hasEnabled) return;

  String enableVal = commandMap["enabled"];

  if (enableVal.equalsIgnoreCase("true")) {
    if (isBlinkingRed) {
      Serial.println("[SYSTEM] Enable signal received. Going GREEN.");
      toggleSystemState();
    }
  } else if (enableVal.equalsIgnoreCase("false")) {
    if (!isBlinkingRed) {
      Serial.println("[SYSTEM] Kill signal received! Going RED.");
      toggleSystemState();
    }
  }
}

// --- Main Program ---

void setup() {
  Serial.begin(115200);
  
  messenger.onMessage(onMessage);
  messenger.begin(WIFI_SSID, WIFI_PASSWORD, BROKER_HOST, BROKER_PORT, GROUP_ID, BoardId);
  Serial.println("Messenger Ready!");

  // Initialize the external LED pins
  pinMode(redPin, OUTPUT);
  pinMode(greenPin, OUTPUT);

  // Start in the safe (Green) state
  setLedSolidGreen();
}

void loop() {
  messenger.loop();

  // Handle the Red LED Blinking
  if (isBlinkingRed) {
    unsigned long currentMillis = millis();
    if (currentMillis - previousMillis >= blinkInterval) {
      previousMillis = currentMillis;     
      
      if (redLedState == LED_ON) {
        redLedState = LED_OFF;
      } else {
        redLedState = LED_ON;
      }
      
      digitalWrite(redPin, redLedState);  
    }
  }

  // Send registration packet every 10 seconds
  if (millis() - lastRegisterMs > 5000 || lastRegisterMs == 0) {
    lastRegisterMs = millis();
    char reg[64];
    snprintf(reg, sizeof(reg), "type=register team_id=%s board_id=%s", GROUP_ID, BoardId);
    messenger.sendToBoard("server", reg);
    Serial.println("Registered with server.");
  }
}