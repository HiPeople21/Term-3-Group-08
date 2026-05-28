#include <Wire.h>
#include "motors.h"

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

bool isRunning = false; 

const int TARGET_DISTANCE_MM = 150; 
// 恢复为直接的 PWM 动力输出 (-800 到 800 范围)
const int BASE_PWM = 550; 

const int DEADBAND_PWM = 60; 


float Kp = 8; 
float Ki = 0; 
float Kd = 20; // 履带车单环特别需要较高的 Kd 来提前“踩刹车”防止撞墙

float integral = 0;
float prevError = 0;
unsigned long lastControlTime = 0;

void setup() {
  Serial.begin(115200); 
  while (!Serial && millis() < 3000); 
  
 
  Serial1.begin(921600); 
  Serial4.begin(921600); 
  
  
  Wire1.begin();
  initMotors(); 
  
  Serial.println("--- Crawler SINGLE-LOOP PID Ready ---");
  Serial.println("Send 'g' to start, 'x' to stop.");
  lastControlTime = millis();
}

void loop() {
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

 
  readTOFSensor(sensor1);
  readTOFSensor(sensor2);

 
  unsigned long currentTime = millis();
  float deltaTime = (currentTime - lastControlTime) / 1000.0; 

 
  if (isRunning && deltaTime >= 0.02) {
    
   
    if (sensor2.current_distance > 0 && (currentTime - sensor2.last_update_time < 100)) {
      

      float error = (float)sensor2.current_distance - TARGET_DISTANCE_MM;
      
     
      integral += error * deltaTime;
      integral = constrain(integral, -300, 300); // 积分上限限制在 300 PWM
      
     
      float derivative = (error - prevError) / deltaTime;
      
     
      float correction = (Kp * error) + (Ki * integral) + (Kd * derivative);
      
     
      if (abs(error) > 5.0) {
        if (correction > 0) correction += DEADBAND_PWM;
        if (correction < 0) correction -= DEADBAND_PWM;
      }

      int cmdLeft = BASE_PWM + correction;
      int cmdRight = BASE_PWM - correction;

      // 限制在硬件安全范围内 (-800 到 800)
      cmdLeft = constrain(cmdLeft, -(800 * 6 / 7.2), (800 * 6 / 7.2));
      cmdRight = constrain(cmdRight, -(800 * 6 / 7.2), (800 * 6 / 7.2));
      
      setLeftTrack(cmdLeft);
      setRightTrack(cmdRight);

     
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
