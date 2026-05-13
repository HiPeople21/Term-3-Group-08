// ═══════════════════════════════════════════════════════════
//  WiFi + Mechanical Kill Switch Demo
//  UCL RAI Robotics Challenge 2026
// ═══════════════════════════════════════════════════════════

#include <WiFi.h>
#include <WiFiUdp.h>
#include "Motoron.h"

// ── WiFi config ──────────────────────────────────────────────
#define WIFI_SSID     "PhaseSpaceNetwork_2.4G"
#define WIFI_PASSWORD "8igMacNet"
const uint16_t UDP_PORT = 4210;

// ── Pin definitions ──────────────────────────────────────────
const int PIN_LED_R   = 5;
const int PIN_LED_G   = 7;
const int PIN_LED_B   = 6;
const int PIN_KILL_SW = 2;

// ── Motor channels on Motoron ────────────────────────────────
const uint8_t MOTOR_LEFT  = 1;
const uint8_t MOTOR_RIGHT = 2;

// ── State ────────────────────────────────────────────────────
bool robotStopped = true;

// ── Mechanical kill switch debounce ──────────────────────────
bool lastButtonState  = HIGH;
unsigned long lastDebounceMs = 0;
const unsigned long DEBOUNCE_DELAY = 50;

// ── LED blink timing ─────────────────────────────────────────
unsigned long lastBlinkMs = 0;
bool ledOn = false;
const unsigned long BLINK_INTERVAL = 400;

// ── Objects ──────────────────────────────────────────────────
MotoronI2C mc;
WiFiUDP udp;
bool wifiOK = false;

// ─────────────────────────────────────────────────────────────

void setLED(bool r, bool g, bool b) {
  digitalWrite(PIN_LED_R, r ? HIGH : LOW);
  digitalWrite(PIN_LED_G, g ? HIGH : LOW);
  digitalWrite(PIN_LED_B, b ? HIGH : LOW);
}

void stopMotors() {
  mc.setSpeed(MOTOR_LEFT,  0);
  mc.setSpeed(MOTOR_RIGHT, 0);
}

void updateLED() {
  if (robotStopped) {
    unsigned long now = millis();
    if (now - lastBlinkMs >= BLINK_INTERVAL) {
      lastBlinkMs = now;
      ledOn = !ledOn;
      setLED(ledOn, false, false); // blink red
    }
  } else {
    setLED(false, true, false); // solid green
  }
}

void checkUDP() {
  if (!wifiOK) return;
  int packetSize = udp.parsePacket();
  if (packetSize == 0) return;

  char buf[64];
  int len = udp.read(buf, sizeof(buf) - 1);
  buf[len] = '\0';

  String msg = String(buf);
  msg.trim();
  Serial.print("[UDP] Received: \"");
  Serial.print(msg);
  Serial.println("\"");

  if (msg.equalsIgnoreCase("STOP")) {
    robotStopped = true;
    stopMotors();
    Serial.println("[UDP] WiFi kill switch → STOPPED");
  } else if (msg.equalsIgnoreCase("START")) {
    robotStopped = false;
    Serial.println("[UDP] WiFi kill switch → RUNNING");
  }
}

void checkMechanicalKillSwitch() {
  bool reading = digitalRead(PIN_KILL_SW);
  if (reading != lastButtonState) lastDebounceMs = millis();

  if ((millis() - lastDebounceMs) > DEBOUNCE_DELAY) {
    if (reading == LOW && lastButtonState == HIGH) {
      robotStopped = !robotStopped;
      if (robotStopped) {
        stopMotors();
        Serial.println("[KSW] Mechanical kill → STOPPED");
      } else {
        Serial.println("[KSW] Mechanical kill → RUNNING");
      }
    }
  }
  lastButtonState = reading;
}

// ─────────────────────────────────────────────────────────────

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000);
  Serial.println("\n[BOOT] Kill Switch Demo Starting...");

  pinMode(PIN_LED_R, OUTPUT);
  pinMode(PIN_LED_G, OUTPUT);
  pinMode(PIN_LED_B, OUTPUT);
  setLED(false, false, false);

  pinMode(PIN_KILL_SW, INPUT_PULLUP);

  Wire.begin();
  mc.reinitialize();
  mc.disableCrc();
  mc.clearResetFlag();
  stopMotors();
  Serial.println("[MOTORON] Initialised");

  Serial.print("[WiFi] Connecting to ");
  Serial.println(WIFI_SSID);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  unsigned long wifiStart = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - wifiStart < 15000) {
    delay(400);
    Serial.print(".");
  }

  if (WiFi.status() == WL_CONNECTED) {
    wifiOK = true;
    udp.begin(UDP_PORT);
    Serial.println();
    Serial.print("[WiFi] Connected! Robot IP: ");
    Serial.println(WiFi.localIP());
    Serial.print("[UDP]  Listening on port ");
    Serial.println(UDP_PORT);
  } else {
    Serial.println("\n[WiFi] FAILED — only mechanical kill switch active");
  }

  Serial.println("[READY] Robot is STOPPED. Send START via UDP or press kill switch.");
}

void loop() {
  checkMechanicalKillSwitch();
  checkUDP();
  updateLED();

  if (!robotStopped) {
    // Your driving code goes here
    // mc.setSpeed(MOTOR_LEFT,  600);
    // mc.setSpeed(MOTOR_RIGHT, 600);
  }

  delay(10);
}