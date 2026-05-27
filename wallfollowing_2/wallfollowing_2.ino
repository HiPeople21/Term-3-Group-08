#include <Wire.h>
#include "motors.h"

// ==========================================
// 1. ToF 传感器配置
// ==========================================
struct TOFSensor {
  Stream& port;
  const char* name;
  uint8_t buffer[16];
  unsigned long current_distance; 
  unsigned long last_update_time; 
};

// 默认右侧贴墙使用 sensor2 (Serial4)
TOFSensor sensor1 = {Serial1, "Sensor 1 (TX0)", {0}, 0, 0};
TOFSensor sensor2 = {Serial4, "Sensor 2 (TX3)", {0}, 0, 0};

// ==========================================
// 2. 状态机与 单环 PID 控制参数
// ==========================================
bool isRunning = false; 

const int TARGET_DISTANCE_MM = 150; 
// 恢复为直接的 PWM 动力输出 (-800 到 800 范围)
const int BASE_PWM = (800 * 6 / 7.2) - 100; 

// 履带车特化：静摩擦死区补偿 (当 PID 修正量太小时，强制叠加这个动力让履带克服阻力动起来)
const int DEADBAND_PWM = 60; 

// 单环 PID 参数 (需在实车上重新整定)
// 注意：因为现在输出是直接加在 PWM 上，数值范围变大了，Kp 需要比双环时小一点
float Kp = 1.5; 
float Ki = 0; 
float Kd = 0.8; // 履带车单环特别需要较高的 Kd 来提前“踩刹车”防止撞墙

float integral = 0;
float prevError = 0;
unsigned long lastControlTime = 0;

void setup() {
  Serial.begin(115200); 
  while (!Serial && millis() < 3000); 
  
  // 初始化 ToF 通信
  Serial1.begin(921600); 
  Serial4.begin(921600); 
  
  // 初始化 I2C 电机驱动
  Wire1.begin();
  initMotors(); 
  
  Serial.println("--- Crawler SINGLE-LOOP PID Ready ---");
  Serial.println("Send 'g' to start, 'x' to stop.");
  lastControlTime = millis();
}

void loop() {
  // ==========================================
  // A. 串口状态机：监听指令
  // ==========================================
  if (Serial.available()) {
    char cmd = Serial.read();
    if (cmd == 'g' || cmd == 'G') {
      isRunning = true;
      integral = 0; // 启动时清空积分，防止猛窜
      lastControlTime = millis();
      Serial.println("[SYSTEM] RUNNING - SINGLE LOOP");
    } 
    else if (cmd == 'x' || cmd == 'X') {
      isRunning = false;
      stopTracks();
      Serial.println("[SYSTEM] STOPPED");
    }
  }

  // ==========================================
  // B. 非阻塞刷新 ToF 数据
  // ==========================================
  readTOFSensor(sensor1);
  readTOFSensor(sensor2);

  // ==========================================
  // C. 核心 单环 PID 控制
  // ==========================================
  unsigned long currentTime = millis();
  float deltaTime = (currentTime - lastControlTime) / 1000.0; 

  // 每 20ms 执行一次计算 (50Hz)
  if (isRunning && deltaTime >= 0.01) {
    
    // 确保右侧 ToF 数据有效且新鲜 (<100ms)
    if (sensor2.current_distance > 0 && (currentTime - sensor2.last_update_time < 100)) {
      
      // 误差计算：当前距离 - 目标距离
      // 太远 > 0 ; 太近 < 0
      float error = (float)sensor2.current_distance - TARGET_DISTANCE_MM;
      
      // 积分与限幅防饱和
      integral += error * deltaTime;
      integral = constrain(integral, -300, 300); // 积分上限限制在 300 PWM
      
      // 微分预测
      float derivative = (error - prevError) / deltaTime;
      
      // 计算 PID 基础修正量
      float correction = (Kp * error) + (Ki * integral) + (Kd * derivative);
      
      // --- 履带特化：死区补偿 ---
      // 如果机器人偏离目标超过 5mm，但 PID 算出来的力量不够克服摩擦力，强行叠加 DEADBAND
      if (abs(error) > 5.0) {
        if (correction > 0) correction += DEADBAND_PWM;
        if (correction < 0) correction -= DEADBAND_PWM;
      }

      // 右侧贴墙转向逻辑：
      // error 为正 (太远) -> correction 为正 -> 左履带加速，右履带减速 -> 向右转靠墙
      // error 为负 (太近) -> correction 为负 -> 左履带减速，右履带加速 -> 向左转远离墙
      int cmdLeft = BASE_PWM + correction;
      int cmdRight = BASE_PWM - correction;

      // 限制在硬件安全范围内 (-800 到 800)
      cmdLeft = constrain(cmdLeft, -800, 800);
      cmdRight = constrain(cmdRight, -800, 800);
      
      setLeftTrack(cmdLeft);
      setRightTrack(cmdRight);

      // --- 串口绘图器专用打印 ---
      Serial.print("TargetDist:150"); 
      Serial.print(", CurrentDist:"); Serial.print(sensor2.current_distance);
      Serial.print(", LeftPWM:"); Serial.print(cmdLeft);
      Serial.print(", RightPWM:"); Serial.println(cmdRight);
      
      prevError = error;
    } 
    else {
      // 如果突然丢失 ToF 信号（比如被遮挡），短暂保持直行，防止乱转
      setLeftTrack(BASE_PWM);
      setRightTrack(BASE_PWM);
    }

    lastControlTime = currentTime;
  }
}

// ==========================================
// D. ToF 数据滑动窗口解析
// ==========================================
void readTOFSensor(TOFSensor& sensor) {
  while (sensor.port.available()) {
    uint8_t c = sensor.port.read();
    
    for (int i = 0; i < 15; i++) {
      sensor.buffer[i] = sensor.buffer[i + 1];
    }
    sensor.buffer[15] = c;
    
    if (sensor.buffer[0] == 0x57 && sensor.buffer[1] == 0x00 && sensor.buffer[2] == 0xFF) {
      sensor.current_distance = sensor.buffer[8] | (sensor.buffer[9] << 8) | (sensor.buffer[10] << 16);
      sensor.last_update_time = millis();
      sensor.buffer[0] = 0x00;
    }
  }
}