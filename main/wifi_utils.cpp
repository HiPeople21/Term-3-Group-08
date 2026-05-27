#include "wifi_utils.h"
#include <map>
#include <MiniMessenger.h>
#include "parser.h"
#include "secrets.h"

static MiniMessenger messenger;
static bool systemEnabled = true;
static unsigned long lastRegisterMs = 0;
static const char* BoardId = "I'm Tired";
bool isFertile = false;

unsigned long lastHeartbeatMs = 0;
const unsigned long HEARTBEAT_TIMEOUT_MS = 1000; // 4x the 250ms server interval

std::map<std::pair<int,int>, std::map<String, String>> grid;

String UIDs[] = {
  "C3DFAA41",
  "671BAB41",
  "855AAB41",
  "70CBAA41",
  "6E54A641",
  "7447AB41",
  "1B0AAB41",
  "418BAB41",
  "43DB2CDD",
  "2802AB41",
  "7074AB41",
  "DF54A941",
  "03CCAA41",
  "1D65AA41",
  "F6B6A941",
  "A42DAB41",
  "F164AB41",
  "A335126A",
  "4A12AB41",
  "685EAB41",
  "E7F7AA41",
  "9C01AB41",
  "E238A941",
  "54C4AA41",
  "8CE5AA41",
  "4E4DAB41",
  "28E3AA41",
  "BD47AB41",
  "F94FAB41",
  "CE9CAA41",
  "060DAB41",
  "6666AA41",
  "B3DA2ADD",
  "D3DDAA41",
  "0D46AB41",
  "A142AB41",
  "5663AB41",
  "0077AB41",
  "48CBAA41",
  "F85EAB41",
  "4FC0AA41",
  "AE55AA41",
  "41AB4141",
  "FCD6AA41",
  "D157AB41",
  "9259AB41",
  "3D84AB41",
  "70D7AA41",
  "B811AB41",
  "3ACEAA41",
  "6C5FAB41",
  "F459AB41",
  "47FAAA41",
  "773DAB41",
  "7451AB41",
  "B493AB41",
  "6D19AB41",
  "8A45AB41",
  "9312AB41",
  "AC5CAB41",
  "E840AB41",
  "F052AB41",
  "10C7AA41",
  "7C88AB41",
  "2A60AB41",
  "E74BA941",
  "C47CAB41",
  "BCCFAA41",
  "07F6AA41",
  "3385AB41",
  "573DAB41",
  "F642AB41",
  "F07EAB41",
  "528AAB41",
  "375CAB41",
  "8145A941",
  "76F0AA41",
  "F63BAB41",
  "9017AB41",
  "390DAB41",
  "1F27AB41",
};

void setupGrid() {
  for (int i = 1; i < 10; i++) {
    for (int j = 1; j < 10; j++) {
      grid[{i, j}]["fertile"] = "NULL";
      grid[{i, j}]["planted"] = "NULL";
      grid[{i, j}]["UID"] = "NULL";
    }
  }
}

bool isSystemEnabled() {
  return systemEnabled;
}

void checkFertility(String tagId) {
  char reg[64];
  snprintf(reg, sizeof(reg), "type=isFertile tag_id=%s board_id=%s", tagId.c_str(), BoardId);
  messenger.sendToBoard("server", reg);
  // Serial.println(reg);
}

void seedPlanted(String tagId) {
  char reg[64];
  snprintf(reg, sizeof(reg), "type=seedPlanted tag_id=%s board_id=%s", tagId.c_str(), BoardId);
  messenger.sendToBoard("server", reg);
  // Serial.println(reg);

  for (auto tile : grid) {
    if (tile.second["UID"].equalsIgnoreCase(tagId)) {
      tile.second["planted"] = "true";
    }
  }
}

void openAirlockA() {
  char reg[64];
  snprintf(reg, sizeof(reg), "type=openAirlockA board_id=%s", BoardId);
  messenger.sendToBoard("server", reg);
}	

void openAirlockB() {
  char reg[64];
  snprintf(reg, sizeof(reg), "type=openAirlockB board_id=%s", BoardId);
  messenger.sendToBoard("server", reg);
}	


void register_bot() {
  lastRegisterMs = millis();
  char reg[64];
  snprintf(reg, sizeof(reg), "type=register team_id=%s board_id=%s", GROUP_ID, BoardId);
  messenger.sendToBoard("server", reg);
  Serial.println("[WiFi] Registered");
}

bool isMessageValid(String str) {
    for (int i = 0; i < str.length(); i++) {
        char c = str[i];
        
        // If the character is not a standard printable character, 
        // and it's NOT a new line (\n) or carriage return (\r)
        if ((c < 32 || c > 126) && c != '\n' && c != '\r') {
            return false; // Found garbage data, reject the whole string
        }
    }
    return true; // String looks completely normal
}

static void onMessage(const MessageMetadata& metadata, const uint8_t* payload, size_t length) {
  String msg = "";
  for (size_t i = 0; i < length; i++) msg += (char)payload[i];
  msg.trim();
  if (msg.length() == 0) return;
  if (!isMessageValid(msg)) return;

  Serial.print("[WiFi] Msg from ");
  Serial.print(metadata.fromBoardId);
  Serial.print(": ");
  Serial.println(msg);

  auto commandMap = parseToMap(msg);

  if (commandMap.count("type") < 1) {
    Serial.println("Message without a type");
    Serial.println(msg);
    return;
  }

  String commandType = commandMap["type"];


  if (commandType == "disable") {
    String enableVal = commandMap["enabled"];
    if (enableVal.equalsIgnoreCase("true")) {
      systemEnabled = true;
      Serial.println("[WiFi] System ENABLED.");
    } else if (enableVal.equalsIgnoreCase("false")) {
      systemEnabled = false;
      Serial.println("[WiFi] System KILLED.");
    }
  } else if (commandType == "emergency") {
      systemEnabled = false;
      Serial.println("[WiFi] Emergency Called.");
    
  } else if (commandType == "isFertileReply") {
    String fertile = commandMap["fertile"];
    if (fertile.equalsIgnoreCase("true")) {

    } else if (fertile.equalsIgnoreCase("false")) {
      
    }
    // Serial.println(msg);

  } else if (commandType == "heartbeat") {
    lastHeartbeatMs = millis(); // Track that server is alive

    if (commandMap["enable"] == "1") {
      systemEnabled = true;
    } else if (commandMap["enable"] == "0") {
      systemEnabled = false;
      Serial.println("[WiFi] Heartbeat Disabled");
    }
  } else if (commandType == "openAirlockReply") {
    // Serial.println(msg);
    
  } else if (commandType == "openAirlock") {

  } else {
    Serial.print("Unknown command: ");
    Serial.println(msg);
  }
  // Serial.println(msg);

}

void initWifi() {
  messenger.onMessage(onMessage);
  messenger.begin(WIFI_SSID, WIFI_PASSWORD, BROKER_HOST, BROKER_PORT, GROUP_ID, BoardId);
  Serial.println("[WiFi] Messenger ready.");

  register_bot();
}

void loopWifi() {
  messenger.loop();
  if (millis() - lastRegisterMs > 5000 || lastRegisterMs == 0) {
    register_bot();
  }

  // if (systemEnabled && (millis() - lastHeartbeatMs > HEARTBEAT_TIMEOUT_MS)) {
  //   systemEnabled = false;
  //   Serial.println("[WiFi] Heartbeat Timeout (Server Connection Lost)");
  // }

}
