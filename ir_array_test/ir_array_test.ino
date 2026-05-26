#include <Wire.h>
#include <QTRSensors.h>
#include "motors.h"
#include "MFRC522_I2C.h"


QTRSensors qtr;
const uint8_t SensorCount = 12;
uint16_t sensorValues[SensorCount];
const uint8_t sensorPins[SensorCount] = {31, 30, 27, 36, 23, 28, 29, 24, 37, 22, 33, 32};
MFRC522_I2C mfrc522(0x28, -1, &Wire1);


float Kp = 1.2;
float Ki = 0.0;
float Kd = 0.0;

const int setpoint     = 6000;
const int baseSpeed    = 800;
const int maxSpeed     = 800;
const int minSpeed     = 0;

float integral  = 0;
float prevError = 0;
unsigned long prevTime = 0;

bool running = false;


float computePID(float position) {
  unsigned long now = millis();
  float dt = (now - prevTime) / 1000.0;
  prevTime = now;
  if (dt <= 0) return 0;

  float error = setpoint - position;


  float P = Kp * error;

  float maxIntegral = 255.0 / max(Ki, 0.0001);
  integral += error * dt;
  integral = constrain(integral, -maxIntegral, maxIntegral);
  float I = Ki * integral;

  float derivative = (error - prevError) / dt;
  float D = Kd * derivative;
  prevError = error;

  return P + I + D;
}

void runLineFollower() {
  uint16_t position = qtr.readLineBlack(sensorValues);
  float correction = computePID(position);

  int leftSpeed  = constrain(baseSpeed - correction, minSpeed, maxSpeed);
  int rightSpeed = constrain(baseSpeed + correction, minSpeed, maxSpeed) * -1;

  setLeftTrack(leftSpeed);
  setRightTrack(rightSpeed);

  Serial.print("Pos: "); Serial.print(position);
  Serial.print(" | Err: "); Serial.print(setpoint - position);
  Serial.print(" | Cor: "); Serial.print(correction);
  Serial.print(" | L: "); Serial.print(leftSpeed);
  Serial.print(" | R: "); Serial.println(rightSpeed);
}

void potentialPlant(){
  Serial.println(sensorValues[SensorCount/2]);
  if(sensorValues[SensorCount/2] >= 140 &&  sensorValues[SensorCount/2] <= 400){
    Serial.println("The middle IR sensor may have passed over a hole");
  }
}

void calibrate() {
  Serial.println("[CAL] Sweep ALL 12 sensors across the line...");
  digitalWrite(LED_BUILTIN, HIGH);
  for (uint16_t i = 0; i < 400; i++) qtr.calibrate();
  digitalWrite(LED_BUILTIN, LOW);
  Serial.println("[CAL] Done.");

  Serial.print("MIN: ");
  for (uint8_t i = 0; i < SensorCount; i++) { Serial.print(qtr.calibrationOn.minimum[i]); Serial.print(' '); }
  Serial.println();
  Serial.print("MAX: ");
  for (uint8_t i = 0; i < SensorCount; i++) { Serial.print(qtr.calibrationOn.maximum[i]); Serial.print(' '); }
  Serial.println();
}

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000);

  Wire1.begin();
  initMotors();
  mfrc522.PCD_Init();

  pinMode(LED_BUILTIN, OUTPUT);

  qtr.setTypeRC();
  qtr.setSensorPins(sensorPins, SensorCount);

  delay(500);
  calibrate();
  delay(1000);

  prevTime = millis();

  Serial.println("[READY] G = Go | X = Stop");
}

void loop() {
  if (Serial.available()) {
    char cmd = Serial.read();
    if (cmd == 'g'  || cmd == 'G')  { running = true;  integral = 0; prevError = 0; Serial.println(">> Running"); }
    else if (cmd == 'x'  || cmd == 'X')  { running = false; stopTracks(); Serial.println(">> Stopped"); }
    else { Serial.print("Unknown: "); Serial.println(cmd); }
  }

  if (running) {
    runLineFollower();
    potentialPlant();
    // M5.Lcd.setCursor(40, 47);
    if (!mfrc522.PICC_IsNewCardPresent() || !mfrc522.PICC_ReadCardSerial()) {
        delay(200);
        return;
    }
    // M5.Lcd.fillRect(42, 47, 320, 20, BLACK);
    for (byte i = 0; i < mfrc522.uid.size; i++) {
        Serial.print(mfrc522.uid.uidByte[i] < 0x10 ? " 0" : " ");
        Serial.print(mfrc522.uid.uidByte[i], HEX);
    }
    // M5.Lcd.println("");
    Serial.println("");
  }
  
}