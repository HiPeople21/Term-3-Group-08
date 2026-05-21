#include "wifi_utils.h"
#include <MiniMessenger.h>
#include "parser.h"
#include "secrets.h"

static MiniMessenger messenger;
static bool systemEnabled = true;
static unsigned long lastRegisterMs = 0;
static const char* BoardId = "Igor-Intator";

bool isSystemEnabled() {
  return systemEnabled;
}

static void onMessage(const MessageMetadata& metadata, const uint8_t* payload, size_t length) {
  String msg = "";
  for (size_t i = 0; i < length; i++) msg += (char)payload[i];
  msg.trim();
  if (msg.length() == 0) return;

  Serial.print("[WiFi] Msg from ");
  Serial.print(metadata.fromBoardId);
  Serial.print(": ");
  Serial.println(msg);

  auto commandMap = parseToMap(msg);

  bool isDisableCmd = commandMap.count("type")   && commandMap["type"].equalsIgnoreCase("disable");
  bool isOperator   = commandMap.count("reason") && commandMap["reason"].equalsIgnoreCase("operator");
  bool hasEnabled   = commandMap.count("enabled") > 0;

  if (!isDisableCmd || !isOperator || !hasEnabled) return;

  String enableVal = commandMap["enabled"];
  if (enableVal.equalsIgnoreCase("true")) {
    systemEnabled = true;
    Serial.println("[WiFi] System ENABLED.");
  } else if (enableVal.equalsIgnoreCase("false")) {
    systemEnabled = false;
    Serial.println("[WiFi] System KILLED.");
  }
}

void initWifi() {
  messenger.onMessage(onMessage);
  messenger.begin(WIFI_SSID, WIFI_PASSWORD, BROKER_HOST, BROKER_PORT, GROUP_ID, BoardId);
  Serial.println("[WiFi] Messenger ready.");
}

void loopWifi() {
  messenger.loop();
  if (millis() - lastRegisterMs > 5000 || lastRegisterMs == 0) {
    lastRegisterMs = millis();
    char reg[64];
    snprintf(reg, sizeof(reg), "type=register team_id=%s board_id=%s", GROUP_ID, BoardId);
    messenger.sendToBoard("server", reg);
  }
}
