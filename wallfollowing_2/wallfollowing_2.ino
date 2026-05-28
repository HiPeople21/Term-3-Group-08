#include <Wire.h>
#include "motors.h"

#define TRIG_FRONT 42
#define ECHO_FRONT 44

struct TOFSensor {
  Stream& port;
  const char* name;
  uint8_t buffer[16];
  unsigned long current_distance; 
  unsigned long last_update_time; 
};

TOFSensor sensor1 = {Serial1, "Sensor 1 (TX0)", {0}, 0, 0};
TOFSensor sensor2 = {Serial4, "Sensor 2 (TX3)", {0}, 0, 0};

bool isRunning = false; 

const int TARGET_DISTANCE_MM = 100; 
const int BASE_PWM = 550; 
const int DEADBAND_PWM = 60; 

float Kp = 3.0; 
float Ki = 0.0; 
float Kd = 2; 

float integral = 0;
float prevError = 0;
unsigned long lastControlTime = 0;

float getFrontDistance() {
  digitalWrite(TRIG_FRONT, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_FRONT, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_FRONT, LOW);
  
  long duration = pulseIn(ECHO_FRONT, HIGH, 6000); 
  
  if (duration == 0) return 999.0; 
  return duration / 58.0;
}

void setup() {
  Serial.begin(115200); 
  while (!Serial && millis() < 3000); 
  
  pinMode(TRIG_FRONT, OUTPUT);
  pinMode(ECHO_FRONT, INPUT);
  
  Serial1.begin(921600); 
  Serial4.begin(921600); 
  Wire1.begin();
  initMotors(); 
  
  Serial.println("--- Crawler SMART PID + AUTO-RESUME Ready ---");
  Serial.println("Send 'g' to start, 'x' to stop.");
  lastControlTime = millis();
}

void loop() {
  if (Serial.available()) {
    char cmd = Serial.read();
    if (cmd == 'g' || cmd == 'G') {
      isRunning = true;
      integral = 0;
      lastControlTime = millis();
      Serial.println("[SYSTEM] RUNNING");
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

  if (isRunning && deltaTime >= - 0.02) {
    
    float frontDist_cm = getFrontDistance();
    
    if (frontDist_cm <= 10.0) {
      stopTracks(); 
      
      integral = 0;
      lastControlTime = currentTime; 
      
      Serial.print("[PAUSED] Obstacle at ");
      Serial.print(frontDist_cm);
      Serial.println(" cm. Waiting for clear path...");
      return; 
    }

    bool seeRightWall = (sensor2.current_distance > 0 && sensor2.current_distance < 300 && (currentTime - sensor2.last_update_time < 100));
    bool seeLeftWall = (sensor1.current_distance > 0 && sensor1.current_distance < 300 && (currentTime - sensor1.last_update_time < 100));

    static float smoothedRight = TARGET_DISTANCE_MM;
    static float smoothedLeft = TARGET_DISTANCE_MM;
    
    if (seeRightWall) smoothedRight = (0.3 * (float)sensor2.current_distance) + (0.7 * smoothedRight);
    if (seeLeftWall)  smoothedLeft = (0.3 * (float)sensor1.current_distance) + (0.7 * smoothedLeft);

    float error = 0.0;
    bool followingRight = true;

    if (seeRightWall && seeLeftWall) {
      if (smoothedRight <= smoothedLeft) {
        followingRight = true;
        error = smoothedRight - TARGET_DISTANCE_MM;
      } else {
        followingRight = false;
        error = smoothedLeft - TARGET_DISTANCE_MM;
      }
    } 
    else if (seeRightWall) {
      followingRight = true;
      error = smoothedRight - TARGET_DISTANCE_MM;
    } 
    else if (seeLeftWall) {
      followingRight = false;
      error = smoothedLeft - TARGET_DISTANCE_MM;
    } 
    else {
      setLeftTrack(BASE_PWM);
      setRightTrack(BASE_PWM);
      integral = 0; 
      lastControlTime = currentTime;
      return; 
    }

    integral += error * deltaTime;
    integral = constrain(integral, -300, 300); 
    
    float derivative = (error - prevError) / deltaTime;
    float correction = (Kp * error) + (Ki * integral) + (Kd * derivative);
    
    if (abs(error) > 5.0) {
      if (correction > 0) correction += DEADBAND_PWM;
      if (correction < 0) correction -= DEADBAND_PWM;
    }

    int cmdLeft = BASE_PWM;
    int cmdRight = BASE_PWM;

    if (followingRight) {
      cmdLeft = BASE_PWM + correction;
      cmdRight = BASE_PWM - correction;
    } else {
      cmdLeft = BASE_PWM - correction;
      cmdRight = BASE_PWM + correction;
    }

    cmdLeft = constrain(cmdLeft, -(800 * 6 / 7.2), (800 * 6 / 7.2));
    cmdRight = constrain(cmdRight, -(800 * 6 / 7.2), (800 * 6 / 7.2));
    
    setLeftTrack(cmdLeft);
    setRightTrack(cmdRight);

    Serial.print(followingRight ? "Wall:RIGHT" : "Wall:LEFT");
    Serial.print(", Front:"); Serial.print(frontDist_cm);
    Serial.print("cm, Dist:"); Serial.print(followingRight ? smoothedRight : smoothedLeft);
    Serial.print("mm, L_PWM:"); Serial.print(cmdLeft);
    Serial.print(", R_PWM:"); Serial.println(cmdRight);
    
    prevError = error;
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
