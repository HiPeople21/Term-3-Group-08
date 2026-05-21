#include <Wire.h>
#include <Motoron.h>

MotoronI2C mc;

#define ENCODER_A 22
#define ENCODER_B 24

volatile long encoderPos = 0; 
long targetPos = 0;

float countsPerRevolution = 1400.0; 
long ticksFor60Degrees = countsPerRevolution / 6; // 250 ticks

unsigned long lastPrintTime = 0;

// --- Linear Control Setting ---
// Set the constant speed you want the motor to travel at.
// If the tracks "overshoot" the target due to momentum, lower this number!
int linearSpeed = 600; 

void setup() {
  Serial.begin(9600);
  while (!Serial);

  Wire1.begin();
  mc.setBus(&Wire1);
  mc.setAddress(18); // Address 0x12
  mc.reinitialize();
  mc.clearResetFlag();

  pinMode(ENCODER_A, INPUT_PULLUP);
  pinMode(ENCODER_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENCODER_A), updateEncoder, CHANGE);

  Serial.println("--- 60-DEGREE LINEAR CONTROLLER ---");
  Serial.println("Math: 250 ticks per jump.");
  Serial.println("Type 'n' to trigger the move.");
}

void loop() {
  if (Serial.available() > 0) {
    char cmd = Serial.read();
    if (cmd == 'n' || cmd == 'N') {
      targetPos += ticksFor60Degrees; 
      Serial.print("\n>>> COMMAND RECEIVED! Moving to Target: ");
      Serial.println(targetPos);
    }
  }

  // Calculate the raw distance to the target
  long error = targetPos - encoderPos;
  int currentSpeedCommand = 0;
  
  // --- LINEAR CONTROL LOGIC ---
  // As long as we are more than 5 ticks away, push power
  if (abs(error) > 5) { 
    if (error > 0) {
      // The target is ahead of us, drive forward at constant speed
      currentSpeedCommand = linearSpeed;
    } else {
      // The target is behind us, drive backward at constant speed
      currentSpeedCommand = -linearSpeed;
    }
  } else {
    // We arrived at the target! 
    currentSpeedCommand = 0; 
  }

  // Send the command to the shield
  mc.setSpeed(2, currentSpeedCommand); 

  // X-Ray Printout
  if (millis() - lastPrintTime > 500) {
    Serial.print("Encoder: "); 
    Serial.print(encoderPos);
    Serial.print(" | Target: "); 
    Serial.print(targetPos);
    Serial.print(" | Speed Output: "); 
    Serial.println(currentSpeedCommand);
    
    lastPrintTime = millis();
  }
}

void updateEncoder() {
  if (digitalRead(ENCODER_A) == digitalRead(ENCODER_B)) {
    encoderPos++; 
  } else {
    encoderPos--; 
  }
}

