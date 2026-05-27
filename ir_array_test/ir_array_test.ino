#include <Wire.h>
#include <QTRSensors.h>
#include "motors.h"
#include "MFRC522_I2C.h"

#define M1A 43
#define M1B 41
#define M2A 47
#define M2B 45

QTRSensors qtr;
MotoronI2C mc;
MFRC522_I2C mfrc522(0x28, -1, &Wire1);

volatile long encoderPosTrack = 0;
volatile long encoderPosPlanter = 0;

const uint8_t SensorCount = 11;

uint16_t sensorValues[SensorCount];

const uint8_t sensorPins[SensorCount] = {
  31, 30, 36,
  23, 28, 29, 24,
  37, 22, 33, 32
};

float Kp = 1;
float Ki = 0;
float Kd = 0;

const int setpoint = 5500;
const int baseSpeed = 800 * 6 / 7.2;
const int maxSpeed = 800 * 6 / 7.2;
const int minSpeed = -800 * 6 / 7.2;
bool TEMPFERTILE = true;

const long ticksToHole = 1355;
const long ticksToPlant = 233;

const unsigned long IR_WINDOW_MS = 1000;

const long junctionCooldownTicks = 200; // tune to junction width in ticks
long lastJunctionTick = -9999;

float integral = 0;
float prevError = 0;
unsigned long prevTime = 0;

bool running = false;

int prevMiddleValue = 0;

long encoderAtIR = 0;
long planterTarget = 0;

unsigned long irDetectedAt = 0;
unsigned long plantingStartedAt = 0;

enum State {
  FOLLOWING,
  WAITING_FOR_RFID,
  DRIVING_TO_HOLE,
  PLANTING,
  JUNCTION_HANDLING
};

State state = FOLLOWING;

void updateTrackEncoder() {
  if (digitalRead(M1A) == digitalRead(M1B)) {
    encoderPosTrack++;
  } else {
    encoderPosTrack--;
  }
}

void updatePlanterEncoder() {
  if (digitalRead(M2A) == digitalRead(M2B)) {
    encoderPosPlanter++;
  } else {
    encoderPosPlanter--;
  }
}

long getTrackEncoder() {
  noInterrupts();
  long value = encoderPosTrack;
  interrupts();
  return value;
}

long getPlanterEncoder() {
  noInterrupts();
  long value = encoderPosPlanter;
  interrupts();
  return value;
}

float computePID(float position) {
  unsigned long now = millis();
  float dt = (now - prevTime) / 1000.0;

  if (dt <= 0) {
    return 0;
  }

  prevTime = now;

  float error = setpoint - position;

  integral += error * dt;

  float derivative = (error - prevError) / dt;

  prevError = error;

  return (Kp * error) + (Ki * integral) + (Kd * derivative);
}

void runLineFollower() {
  uint16_t position = qtr.readLineBlack(sensorValues);

  float correction = computePID(position);

  int leftSpeed = constrain(baseSpeed - correction, minSpeed, maxSpeed);
  int rightSpeed = constrain((baseSpeed + correction), minSpeed, maxSpeed);

  setLeftTrack(leftSpeed);
  setRightTrack(rightSpeed);
}

bool checkForHole() {
  int middleValue = sensorValues[SensorCount / 2];

  bool inHole = middleValue >= 100 && middleValue <= 400;

  bool wasOutside =
    prevMiddleValue < 100 ||
    prevMiddleValue > 400;

  prevMiddleValue = middleValue;

  return inHole && wasOutside;
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

bool isJunction() {
  uint16_t position = qtr.readLineBlack(sensorValues);



  if (sensorValues[0] > 800 && sensorValues[10] > 800) return true;
  else return false;

}

void driveStraight(int speed) {
  setLeftTrack(speed);
  setRightTrack(speed);
}

void resetPID() {
  integral = 0;
  prevError = 0;
  prevTime = millis();
}

void setup() {
  Serial.begin(115200);

  Wire1.begin();

  initMotors();

  mfrc522.PCD_Init();

  pinMode(LED_BUILTIN, OUTPUT);

  qtr.setTypeRC();
  qtr.setSensorPins(sensorPins, SensorCount);

  delay(500);

  calibrate();

  mc.setBus(&Wire1);
  mc.setAddress(18);
  mc.reinitialize();
  mc.clearResetFlag();

  pinMode(M1A, INPUT_PULLUP);
  pinMode(M1B, INPUT_PULLUP);

  pinMode(M2A, INPUT_PULLUP);
  pinMode(M2B, INPUT_PULLUP);

  attachInterrupt(
    digitalPinToInterrupt(M1A),
    updateTrackEncoder,
    CHANGE
  );

  attachInterrupt(
    digitalPinToInterrupt(M2A),
    updatePlanterEncoder,
    CHANGE
  );

  prevTime = millis();

  Serial.println("Ready");
}

void loop() {
  if (Serial.available()) {
    char cmd = Serial.read();

    if (cmd == 'g' || cmd == 'G') {
      running = true;
      resetPID();
      state = FOLLOWING;
      Serial.println("Running");
    }

    if (cmd == 'x' || cmd == 'X') {
      running = false;
      stopTracks();
      Serial.println("Stopped");
    }
  }

  if (!running) {
    return;
  }
  Serial.println(state);
  switch (state) {
    
    case FOLLOWING: {
      runLineFollower();

      if (checkForHole()) {
        irDetectedAt = millis();
        encoderAtIR = getTrackEncoder();
        state = WAITING_FOR_RFID;
        Serial.println("Hole detected");
        break;
      } else if (isJunction()) {
        stopTracks();
        state = JUNCTION_HANDLING;
        Serial.println("Junction detected");
        break;
      } else {
        Serial.print("Following Line, not at junction");
        break;
      }
    }

    case JUNCTION_HANDLING: {
      driveStraight(600 * 6 / 7.2);
      delay(200);
      stopTracks();

      lastJunctionTick = getTrackEncoder(); // record tick after crossing so cooldown is relative to exit point
      resetPID();

      state = FOLLOWING;
      Serial.println("Junction cleared");
      break;
    }

    case WAITING_FOR_RFID: {
      if(isJunction()){
        driveStraight(600 * 6 / 7.2);
        Serial.println("going straight to rfid");
      } else {
        runLineFollower();
      }
      if (millis() - irDetectedAt > IR_WINDOW_MS) {
        state = FOLLOWING;
        Serial.println("RFID timeout, false positive ir detection");
        break;
      }

      if (mfrc522.PICC_IsNewCardPresent() && mfrc522.PICC_ReadCardSerial()) {
        mfrc522.PICC_HaltA();
        mfrc522.PCD_StopCrypto1();
        state = DRIVING_TO_HOLE;
        Serial.println("RFID confirmed");
      }

      break;
    }

    case DRIVING_TO_HOLE: {
      long currentTicks = getTrackEncoder();
      long tickTarget = encoderAtIR + ticksToHole;
      long encerror = tickTarget - currentTicks;

      if (encerror > 5) {
        driveStraight(600 * 6 / 7.2);
        Serial.print("going straight to hole");
      } else {
        stopTracks();
        planterTarget = getPlanterEncoder() + ticksToPlant;
        plantingStartedAt = millis();
        state = PLANTING;
        Serial.println("At planting position");
      }

      break;
    }

    case PLANTING: {
      long planterPos = getPlanterEncoder();

      if (abs(planterPos - planterTarget) > 5) {
        setPlanter(500 * 6 / 7.2);
      } else {
        setPlanter(0);
        delay(500);
        resetPID();
        state = FOLLOWING;
        Serial.println("Plant complete");
      }

      break;
    }
  }
}
