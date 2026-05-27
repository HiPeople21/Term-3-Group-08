#include <Wire.h>
#include <MiniMessenger.h>
#include "motors.h"
#include "parser.h"
#include "secrets.h"

// --- LED pins ---
#define LED_RED_PIN   38
#define LED_GREEN_PIN 40

// --- State ---
static bool isDisabled = false;
static unsigned long previousMillis = 0;
static const long blinkInterval = 500;
static int redLedState = LOW;

static MiniMessenger messenger;
static unsigned long lastRegisterMs = 0;
static const char* BoardId = "bot";

static const int trackSpeed = 800;

// --- Sequence state machine ---
static const unsigned long LEG_DURATION = 1500;
static bool     seqRunning   = false;
static int      seqIteration = 0;
static int      seqLeg       = 0;
static unsigned long seqLegStart = 0;

// --- LED helpers ---
void setLedGreen() {
  digitalWrite(LED_RED_PIN,   LOW);
  digitalWrite(LED_GREEN_PIN, HIGH);
}

void setDisabled(bool disabled) {
  if (disabled == isDisabled) return;
  isDisabled = disabled;
  if (isDisabled) {
    stopTracks();
    seqRunning = false;
    digitalWrite(LED_GREEN_PIN, LOW);
    redLedState = HIGH;
    digitalWrite(LED_RED_PIN, HIGH);
    previousMillis = millis();
    Serial.println("[Kill] System DISABLED.");
  } else {
    setLedGreen();
    Serial.println("[Kill] System ENABLED.");
  }
}

// --- WiFi message callback ---
void onMessage(const MessageMetadata& metadata, const uint8_t* payload, size_t length) {
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
  if      (enableVal.equalsIgnoreCase("true"))  setDisabled(false);
  else if (enableVal.equalsIgnoreCase("false")) setDisabled(true);
}

// --- Non-blocking sequence state machine ---
void applyLeg(int leg) {
  switch (leg) {
    case 0: setRightTrack(trackSpeed);  setLeftTrack(trackSpeed);  break; // forward
    case 1: setRightTrack(-trackSpeed); setLeftTrack(-trackSpeed); break; // reverse
    case 2: setRightTrack(trackSpeed);  setLeftTrack(-trackSpeed); break; // left
    case 3: setRightTrack(-trackSpeed); setLeftTrack(trackSpeed);  break; // right
  }
}

void startSequence() {
  if (isDisabled) return;
  seqRunning   = true;
  seqIteration = 0;
  seqLeg       = 0;
  seqLegStart  = millis();
  applyLeg(0);
}

void updateSequence() {
  if (!seqRunning) return;

  if (isDisabled) { seqRunning = false; return; }

  if (millis() - seqLegStart < LEG_DURATION) return;

  seqLeg++;
  if (seqLeg >= 4) {
    seqLeg = 0;
    seqIteration++;
    if (seqIteration >= 10) {
      stopTracks();
      seqRunning = false;
      Serial.println("Sequence done");
      return;
    }
  }

  seqLegStart = millis();
  applyLeg(seqLeg);
}

// -----------------------------------------------------------------------

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000);

  Wire1.begin();
  initMotors();
  Serial.println("[Motors] Ready.");

  pinMode(LED_RED_PIN,   OUTPUT);
  pinMode(LED_GREEN_PIN, OUTPUT);
  setLedGreen();

  messenger.onMessage(onMessage);
  messenger.begin(WIFI_SSID, WIFI_PASSWORD, BROKER_HOST, BROKER_PORT, GROUP_ID, BoardId);
  Serial.println("[WiFi] Messenger ready.");

  Serial.println("=== ELECTRONICS TEST ===");
  Serial.println("W/S/A/D = Fwd/Rev/Left/Right | X = Stop | Z = Sequence");
}

void loop() {
  messenger.loop();
  updateSequence();

  // Blink red LED while disabled
  if (isDisabled) {
    unsigned long now = millis();
    if (now - previousMillis >= (unsigned long)blinkInterval) {
      previousMillis = now;
      redLedState = (redLedState == HIGH) ? LOW : HIGH;
      digitalWrite(LED_RED_PIN, redLedState);
    }
  }

  // Registration heartbeat every 5 s
  if (millis() - lastRegisterMs > 1000 || lastRegisterMs == 0) {
    lastRegisterMs = millis();
    char reg[64];
    snprintf(reg, sizeof(reg), "type=register team_id=%s board_id=%s", GROUP_ID, BoardId);
    messenger.sendToBoard("server", reg);
  }

  // Serial motor control — gated on enabled state
  if (isDisabled || !Serial.available()) return;

  char cmd = Serial.read();
  if      (cmd == 'w' || cmd == 'W') { setRightTrack(trackSpeed);  setLeftTrack(trackSpeed);  Serial.println("Forward"); }
  else if (cmd == 's' || cmd == 'S') { setRightTrack(-trackSpeed); setLeftTrack(-trackSpeed); Serial.println("Reverse"); }
  else if (cmd == 'a' || cmd == 'A') { setRightTrack(trackSpeed);  setLeftTrack(-trackSpeed); Serial.println("Left"); }
  else if (cmd == 'd' || cmd == 'D') { setRightTrack(-trackSpeed); setLeftTrack(trackSpeed);  Serial.println("Right"); }
  else if (cmd == 'x' || cmd == 'X') { stopTracks(); seqRunning = false;                      Serial.println("Stop"); }
  else if (cmd == 'z' || cmd == 'Z') { startSequence();                                        Serial.println("Sequence started"); }
}
