#include <Wire.h>
#include <MFRC522_I2C.h>
#include "motors.h"
#include "sensors.h"
#include "wifi_utils.h"

// --- RFID (Wire1, I2C address 0x28) ---
MFRC522_I2C mfrc522(0x28, -1, &Wire1);

// --- Kill Switch Pins ---
#define KILL_BUTTON_PIN 39
#define LED_RED_PIN     38
#define LED_GREEN_PIN   40

// --- Kill Switch State ---
static bool isKilledLocal    = false;
static int  killBtnState     = HIGH;
static int  lastBtnState     = HIGH;
static unsigned long lastDebounceTime = 0;
static const unsigned long debounceDelay = 50;

// --- LED Blink State ---
static unsigned long previousMillis = 0;
static const long blinkInterval = 500;
static bool redLedOn = false;

// --- Revival Button (pin 48) ---
#define REVIVE_BUTTON_PIN 48
static int  reviveBtnState  = HIGH;
static int  lastReviveState = HIGH;
static unsigned long lastReviveDebounce = 0;

// --- Motor parameters ---
const int baseSpeed = 500 * 6 / 7.2;

bool running = true;

// ==========================================
// --- Task 4 专属硬编码变量 ---
// ==========================================
int t4Step = 0;        
long startTicks = 0;   

// 【核心参数】等你测出真实数值后，修改这里的值 (目前用 1000 占位)
const long TICKS_PER_NODE = 2700; 
// ==========================================

enum Stage {
  TASK_4_OPEN_FIELD,
  CALIBRATION, // <--- 专属测量模式
  DONE
};

Stage stage = TASK_4_OPEN_FIELD; // 当前设置为测量模式

enum State {
  FOLLOWING,
  TURNING
};

State turnReturnState = FOLLOWING;
State state = FOLLOWING;

// -----------------------------------------------------------------------

void revive() {
  // Placeholder
}

void checkReviveButton() {
  int reading = digitalRead(REVIVE_BUTTON_PIN);
  if (reading != lastReviveState) lastReviveDebounce = millis();
  lastReviveState = reading;
  if ((millis() - lastReviveDebounce) > debounceDelay && reading != reviveBtnState) {
    reviveBtnState = reading;
    if (reviveBtnState == LOW) revive();
  }
}

void updateLED() {
  bool killed = isKilledLocal || !isSystemEnabled();
  if (!killed) {
    digitalWrite(LED_RED_PIN,   LOW);
    digitalWrite(LED_GREEN_PIN, HIGH);
    redLedOn = false;
    return;
  }
  unsigned long now = millis();
  if (now - previousMillis >= (unsigned long)blinkInterval) {
    previousMillis = now;
    redLedOn = !redLedOn;
    digitalWrite(LED_RED_PIN,   redLedOn ? HIGH : LOW);
    digitalWrite(LED_GREEN_PIN, LOW);
  }
}

void checkKillButton() {
  int reading = digitalRead(KILL_BUTTON_PIN);
  if (reading != lastBtnState) lastDebounceTime = millis();
  lastBtnState = reading;
  if ((millis() - lastDebounceTime) > debounceDelay && reading != killBtnState) {
    killBtnState = reading;
    if (killBtnState == LOW) {
      isKilledLocal = !isKilledLocal;
      if (isKilledLocal) stopTracks();
    }
  }
}

void driveStraight(int speed) {
  setLeftTrack(speed);
  setRightTrack(speed);
}

void initTurn(float degrees, State returnState) {
  turnReturnState = returnState;
  startTurn(degrees);
  state = TURNING;
}

// -----------------------------------------------------------------------

void setup() {
  Serial.begin(115200);
  
  Wire1.begin();
  mfrc522.PCD_Init();
  initMotors();
  initSensors();
  
  // 删除了 initIRArray(); 秒开机！

  pinMode(LED_RED_PIN,      OUTPUT);
  pinMode(LED_GREEN_PIN,    OUTPUT);
  pinMode(KILL_BUTTON_PIN,  INPUT_PULLUP);
  pinMode(REVIVE_BUTTON_PIN, INPUT_PULLUP);
  digitalWrite(LED_RED_PIN,   LOW);
  digitalWrite(LED_GREEN_PIN, HIGH);

  initWifi();
  setupGrid();
  register_bot();

  // ==============================================================
  // 【模式切换开关】
  // 目前处于：测量校准模式 (测完以后改成 TASK_4_OPEN_FIELD 即可)
  // ==============================================================
  
  stage = CALIBRATION; 
  state = FOLLOWING;
  t4Step = 0;
  
  // ==============================================================
}

void loop() {
  loopWifi();
  checkKillButton();
  checkReviveButton();
  updateLED();

  bool killed = isKilledLocal || !isSystemEnabled();
  static bool wasPreviouslyKilled = false;
  if (killed && !wasPreviouslyKilled) {
    stopTracks();
    stopPlanter();
    wasPreviouslyKilled = true;
  } else if (!killed) {
    wasPreviouslyKilled = false;
  }

  // 统一处理转弯
  if (!killed && state == TURNING) {
    if (updateTurn()) {
      state = turnReturnState;
    }
  }

  if (running && !killed) {
    switch (stage){
      
      // =========================================
      // 专属测量校准模式
      // =========================================
      case CALIBRATION: {
        stopTracks(); // 强制停车，确保安全
        
        static unsigned long lastPrintTime = 0;
        if (millis() - lastPrintTime > 200) {
          lastPrintTime = millis();
          long currentTicks = getTrackEncoder();
          Serial.print("【校准中】当前 Encoder 数值: ");
          Serial.println(currentTicks);
        }
        break;
      }

      // =========================================
      // Task 4: 盲走航位推算 (Open-Field Dead Reckoning)
      // =========================================
      case TASK_4_OPEN_FIELD: {
        
        // 顺手读取 RFID，防止底层 I2C 堵塞
        if (mfrc522.PICC_IsNewCardPresent() && mfrc522.PICC_ReadCardSerial()) {
          mfrc522.PICC_HaltA();
          mfrc522.PCD_StopCrypto1();
        }

        if (state == TURNING) break;

        long currentTicks = getTrackEncoder();

        switch (t4Step) {
          case 0: // 初始化并开始直行 (目标2格)
            startTicks = currentTicks;
            driveStraight(baseSpeed);
            t4Step = 1;
            break;

          case 1: // 等待前进2格完成
            if (abs(currentTicks - startTicks) >= (2 * TICKS_PER_NODE)) {
              stopTracks();
              delay(200); 
              initTurn(90.0, FOLLOWING); // 右转
              t4Step = 2;
            }
            break;

          case 2: // 右转结束，开始直行 (目标1格)
            if (state == FOLLOWING) {
              startTicks = currentTicks;
              driveStraight(baseSpeed);
              t4Step = 3;
            }
            break;

          case 3: // 等待前进1格完成
            if (abs(currentTicks - startTicks) >= (1 * TICKS_PER_NODE)) {
              stopTracks();
              delay(200);
              initTurn(-90.0, FOLLOWING); // 左转
              t4Step = 4;
            }
            break;

          case 4: // 左转结束，开始直行 (目标2格)
            if (state == FOLLOWING) {
              startTicks = currentTicks;
              driveStraight(baseSpeed);
              t4Step = 5;
            }
            break;

          case 5: // 等待最后2格完成
            if (abs(currentTicks - startTicks) >= (2 * TICKS_PER_NODE)) {
              stopTracks();
              stage = DONE; // 任务结束
            }
            break;
        }
        break;
      }

      case DONE: {
        stopTracks();
        break;
      }
    }
  } else if (!killed) {
    // manual mode planter code if needed
  }
}