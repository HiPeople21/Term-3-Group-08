#include "wifi_utils.h"
#include <map>
#include <MiniMessenger.h>
#include "parser.h"
#include "secrets.h"

static MiniMessenger messenger;
static bool systemEnabled = true;
static bool emergencyFlag = false;
static int serverTimeLeft = -1;
static unsigned long lastRegisterMs = 0;
static const char* BoardId = "TARS";
bool isFertile = false;

unsigned long lastHeartbeatMs = 0;
const unsigned long HEARTBEAT_TIMEOUT_MS = 10000;

std::map<std::pair<int,int>, std::map<String, String>> grid;

// 9x9 occupancy map: 0=Sterile, 1=Fertile, 2=Seeded, 3=Unexplored
static uint8_t gridState[9][9];

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
  memset(gridState, 3, sizeof(gridState));
}

static void (*fertilityCallback)(bool, int, int) = nullptr;

void setFertilityCallback(void (*cb)(bool fertile, int x, int y)) {
  fertilityCallback = cb;
}

bool isSystemEnabled() {
  return systemEnabled;
}

void checkFertility(String tagId) {
  char reg[64];
  snprintf(reg, sizeof(reg), "type=isFertile tag_id=%s board_id=%s", tagId.c_str(), BoardId);
  messenger.sendToBoard("server", reg);
}

void seedPlanted(String tagId) {
  char reg[64];
  snprintf(reg, sizeof(reg), "type=seedPlanted tag_id=%s board_id=%s", tagId.c_str(), BoardId);
  messenger.sendToBoard("server", reg);
}

void openAirlock(String tagId, char airlock) {
  char reg[128];
  snprintf(reg, sizeof(reg), "type=openAirlock airlock=%c tag_id=%s board_id=%s", airlock, tagId.c_str(), BoardId);
  messenger.sendToBoard("server", reg);
}

void reviveRequest(int target_team, String target_board) {
  char reg[64];
  snprintf(reg, sizeof(reg), "type=reviveRequest target_team=%d target_board=%s", target_team, target_board.c_str());
  messenger.sendToBoard("server", reg);
}

void getMap() {
  char reg[64];
  snprintf(reg, sizeof(reg), "type=getMap board_id=%s", BoardId);
  messenger.sendToBoard("server", reg);
}

void register_bot() {
  lastRegisterMs = millis();
  char reg[64];
  snprintf(reg, sizeof(reg), "type=register team_id=%s board_id=%s", GROUP_ID, BoardId);
  messenger.sendToBoard("server", reg);
}

bool isMessageValid(String str) {
    for (unsigned int i = 0; i < str.length(); i++) {
        char c = str[i];
        if ((c < 32 || c > 126) && c != '\n' && c != '\r') {
            return false;
        }
    }
    return true;
}

static void onMessage(const MessageMetadata& metadata, const uint8_t* payload, size_t length) {
  String msg = "";
  for (size_t i = 0; i < length; i++) msg += (char)payload[i];
  msg.trim();
  if (msg.length() == 0) return;
  if (!isMessageValid(msg)) return;

  // 1. Team Status Broadcast (6 bytes)
  //    [0]=queueExit [1]=airlockBBusy [2]=queueEnter
  //    [3]=airlockABusy [4]=emergency [5]=reEntryRequested
  if (length == 6) {
    if (payload[4] == 1) {
      emergencyFlag = true;
      systemEnabled = false;
    } else if (payload[4] == 0 && emergencyFlag) {
      emergencyFlag = false;
      systemEnabled = true;
    }
    return;
  }

  // 2. Occupancy Map (21 bytes binary)
  //    81 cells × 2 bits = 162 bits, packed little-endian into 21 bytes
  //    Cell states: 0=Sterile, 1=Fertile, 2=Seeded, 3=Unexplored
  if (length == 21) {
    for (int i = 0; i < 81; i++) {
      int byteIdx = (i * 2) / 8;
      int bitOff  = (i * 2) % 8;
      uint8_t state = (payload[byteIdx] >> bitOff) & 0x03;
      gridState[i / 9][i % 9] = state;
    }
    return;
  }

  // 3. Text messages
  auto commandMap = parseToMap(msg);

  if (commandMap.count("type") < 1) {
    return;
  }

  String commandType = commandMap["type"];

  if (commandType == "disable") {
    String enableVal = commandMap["enabled"];
    if (enableVal.equalsIgnoreCase("true")) {
      systemEnabled = true;
    } else if (enableVal.equalsIgnoreCase("false")) {
      systemEnabled = false;
    }

  } else if (commandType == "emergency") {
    String enabled = commandMap["enabled"];
    if (enabled.equalsIgnoreCase("true")) {
      emergencyFlag = true;
      systemEnabled = false;
    } else {
      emergencyFlag = false;
      systemEnabled = true;
    }

  } else if (commandType == "isFertileReply") {
    String fertile = commandMap["fertile"];
    String planted = commandMap["planted"];
    bool canPlant = fertile.equalsIgnoreCase("true") && !planted.equalsIgnoreCase("true");

    int gx = 0, gy = 0;
    if (commandMap.count("x") && commandMap.count("y")) {
      gx = commandMap["x"].toInt();
      gy = commandMap["y"].toInt();
      if (gx >= 1 && gx <= 9 && gy >= 1 && gy <= 9) {
        if (planted.equalsIgnoreCase("true"))       gridState[gx-1][gy-1] = 2; // Seeded
        else if (fertile.equalsIgnoreCase("true"))   gridState[gx-1][gy-1] = 1; // Fertile
        else                                         gridState[gx-1][gy-1] = 0; // Sterile
      }
    }
    if (fertilityCallback) fertilityCallback(canPlant, gx, gy);

  } else if (commandType == "heartbeat") {
    lastHeartbeatMs = millis();

    if (commandMap["enable"] == "1") {
      systemEnabled = true;
    } else if (commandMap["enable"] == "0") {
      systemEnabled = false;
    }

    if (commandMap.count("time_left")) {
      serverTimeLeft = commandMap["time_left"].toInt();
    }

  } else if (commandType == "openAirlockReply") {
    // accepted=true/false — door mechanism is automatic, no action needed

  } else if (commandType == "distress") {
    // Info about stranded robots — could be used for revival missions

  } else if (commandType == "reviveReply") {
    // Confirmation of successful rescue
  }
}

void initWifi() {
  messenger.onMessage(onMessage);
  messenger.begin(WIFI_SSID, WIFI_PASSWORD, BROKER_HOST, BROKER_PORT, GROUP_ID, BoardId);

  register_bot();
  lastHeartbeatMs = millis();
}

void loopWifi() {
  messenger.loop();
  if (millis() - lastRegisterMs > 5000 || lastRegisterMs == 0) {
    register_bot();
  }

  if (systemEnabled && (millis() - lastHeartbeatMs > HEARTBEAT_TIMEOUT_MS)) {
    systemEnabled = false;
  }
}

bool isEmergency() { return emergencyFlag; }
void clearEmergency() { emergencyFlag = false; }
int getTimeLeft() { return serverTimeLeft; }

uint8_t getCellState(int x, int y) {
  if (x < 1 || x > 9 || y < 1 || y > 9) return 3;
  return gridState[x-1][y-1];
}
