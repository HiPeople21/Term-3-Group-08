#include <Wire.h>
#include "motors.h"

// Ultrasonic pins
#define TRIG_FRONT 44
#define ECHO_FRONT 42

struct TOFSensor {
  Stream& port;
  const char* name;
  uint8_t buffer[16];
  unsigned long current_distance;
  unsigned long last_update_time;
};

// sensor1 = Left, sensor2 = Right
TOFSensor sensor1 = {Serial1, "Sensor 1 (TX0)", {0}, 0, 0};
TOFSensor sensor2 = {Serial4, "Sensor 2 (TX3)", {0}, 0, 0};

bool isRunning = false;

// Control parameters
const int TARGET_DISTANCE_MM = 150;
const int BASE_PWM = 550;
const int DEADBAND_PWM = 60;

// PID constants
float Kp = 4.0;
float Ki = 0.0;
float Kd = 7.0;

float integral = 0;
float prevError = 0;
unsigned long lastControlTime = 0;

// Get front distance with timeout to prevent blocking
float getFrontDistance() {
  digitalWrite(TRIG_FRONT, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_FRONT, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_FRONT, LOW);

  long duration = pulseIn(ECHO_FRONT, HIGH, 6000);

  if (duration == 0) return 999.0; // Timeout, path is clear
  return duration / 58.0;
}

void setup() {
  // Serial.begin(115200);

  delay(2000);

  pinMode(TRIG_FRONT, OUTPUT);
  pinMode(ECHO_FRONT, INPUT);

  Serial1.begin(921600);
  Serial4.begin(921600);
  Wire1.begin();

  delay(500);
  initMotors();

  // Serial.println("System Ready.");
  isRunning = true;
  lastControlTime = millis();
}

void loop() {
  readTOFSensor(sensor1);
  readTOFSensor(sensor2);

  unsigned long currentTime = millis();
  float deltaTime = (currentTime - lastControlTime) / 1000.0;

  // Run control loop at 50Hz (20ms)
  if (isRunning && deltaTime >= 0.02) {

    float frontDist_cm = getFrontDistance();

    // Obstacle avoidance: pause and wait
    if (frontDist_cm <= 10.0) {
      stopTracks();
      integral = 0;
      prevError = 0;
      lastControlTime = currentTime;
      Serial.println("[STOP] Front obstacle");
      return;
    }

    // Check valid walls (< 500mm)
    bool seeRightWall = (sensor2.current_distance > 0 && sensor2.current_distance < 500 && (currentTime - sensor2.last_update_time < 100));
    bool seeLeftWall = (sensor1.current_distance > 0 && sensor1.current_distance < 500 && (currentTime - sensor1.last_update_time < 100));

    static float smoothedRight = TARGET_DISTANCE_MM;
    static float smoothedLeft = TARGET_DISTANCE_MM;

    // Apply low-pass filter
    if (seeRightWall) smoothedRight = (0.3 * (float)sensor2.current_distance) + (0.7 * smoothedRight);
    if (seeLeftWall)  smoothedLeft = (0.3 * (float)sensor1.current_distance) + (0.7 * smoothedLeft);

    float error = 0.0;
    bool followingRight = true;

    // Smart wall selection
    if (seeRightWall && seeLeftWall) {
      // Follow the closer wall
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
      // No walls detected, go straight
      setLeftTrack(BASE_PWM);
      setRightTrack(BASE_PWM);
      integral = 0;
      lastControlTime = currentTime;
      return;
    }

    // PID calculation
    integral += error * deltaTime;
    integral = constrain(integral, -300, 300);

    float derivative = (error - prevError) / deltaTime;
    float correction = (Kp * error) + (Ki * integral) + (Kd * derivative);

    if (error > 0.0f && derivative < 0.0f) {
      correction = min(correction, 0.0f);
    }

    // Deadband compensation for track friction
    if (abs(error) > 5.0) {
      if (correction > 0) correction += DEADBAND_PWM;
      if (correction < 0) correction -= DEADBAND_PWM;
    }

    int cmdLeft = BASE_PWM;
    int cmdRight = BASE_PWM;

    // Steering logic
    if (followingRight) {
      cmdLeft = BASE_PWM + correction;
      cmdRight = BASE_PWM - correction;
    } else {
      cmdLeft = BASE_PWM - correction;
      cmdRight = BASE_PWM + correction;
    }

    const int MIN_FORWARD_PWM = 100;
    cmdLeft = constrain(cmdLeft, MIN_FORWARD_PWM, (int)(800 * 6 / 7.2));
    cmdRight = constrain(cmdRight, MIN_FORWARD_PWM, (int)(800 * 6 / 7.2));

    setLeftTrack(cmdLeft);
    setRightTrack(cmdRight);

    // Debug info
    // Serial.print(followingRight ? "Wall:RIGHT" : "Wall:LEFT");
    // Serial.print(", Front:"); Serial.print(frontDist_cm);
    // Serial.print("cm, Dist:"); Serial.print(followingRight ? smoothedRight : smoothedLeft);
    // Serial.print("mm, L_PWM:"); Serial.print(cmdLeft);
    // Serial.print(", R_PWM:"); Serial.println(cmdRight);

    prevError = error;
    lastControlTime = currentTime;
  }
}

// Parse ToF data from serial buffer
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
