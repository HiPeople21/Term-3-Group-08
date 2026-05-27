#include <Wire.h>
#include <PID_v1.h>
#include "motors.h" 


struct TOFSensor {
  Stream& port;
  const char* name;
  uint8_t buffer[16];
};

TOFSensor sensorLeft = {Serial1, "Left_ToF", {0}};  
TOFSensor sensorRight = {Serial4, "Right_ToF", {0}}; 

unsigned long distLeft = 0;
unsigned long distRight = 0;



const int ENCODER_L_A = 2; 
const int ENCODER_R_A = 3; 

volatile long encoderTicksL = 0;
volatile long encoderTicksR = 0;


const int interval = 50; 
unsigned long previousMillis = 0;


double inputL, outputL, setpointL;
double inputR, outputR, setpointR;


double baseSpeed = 50.0; 


double Kp = 5.0, Ki = 10.0, Kd = 0.1; 

PID pidLeft(&inputL, &outputL, &setpointL, Kp, Ki, Kd, DIRECT);
PID pidRight(&inputR, &outputR, &setpointR, Kp, Ki, Kd, DIRECT);


double wallKp = 0.3; 

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000);

  Serial.println("--- Starting Ramp Traverse System ---");

 
  Serial1.begin(921600); 
  Serial4.begin(921600); 

  Wire1.begin();
  initMotors();
  stopTracks();


  pinMode(ENCODER_L_A, INPUT_PULLUP);
  pinMode(ENCODER_R_A, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENCODER_L_A), countEncoderL, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCODER_R_A), countEncoderR, RISING);

 
  pidLeft.SetMode(AUTOMATIC);
  pidRight.SetMode(AUTOMATIC);
  

  pidLeft.SetOutputLimits(-800, 800); 
  pidRight.SetOutputLimits(-800, 800);
  
  Serial.println("[System] Ready. Waiting for command...");
}

void loop() {

  readTOFSensor(sensorLeft, distLeft);
  readTOFSensor(sensorRight, distRight);


  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;

   
    noInterrupts(); 
    long currentTicksL = encoderTicksL;
    long currentTicksR = encoderTicksR;
    encoderTicksL = 0; 
    encoderTicksR = 0;
    interrupts(); 

    inputL = abs(currentTicksL);
    inputR = abs(currentTicksR);

  
    double correction = 0;
   
    if (distLeft > 50 && distRight > 50 && distLeft < 400 && distRight < 400) {
      double error = (double)distLeft - (double)distRight;
      
      correction = error * wallKp; 
    }

    
    correction = constrain(correction, -20.0, 20.0);

    setpointL = baseSpeed + correction;
    setpointR = baseSpeed - correction;

   
    pidLeft.Compute();
    pidRight.Compute();

  
    
    setLeftTrack((int)outputL);
    setRightTrack((int)outputR);

    
     Serial.print("SetL:"); Serial.print(setpointL);
     Serial.print(" InL:"); Serial.print(inputL);
     Serial.print(" OutL:"); Serial.print(outputL);
     Serial.print(" | DistL:"); Serial.print(distLeft);
     Serial.print(" DistR:"); Serial.println(distRight);
  }

  
  if (Serial.available()) {
    char cmd = Serial.read();
    if (cmd == 'x' || cmd == 'X') {
      stopTracks();
      baseSpeed = 0; 
      Serial.println("STOPPED");
    }
  }
}


void readTOFSensor(TOFSensor& sensor, unsigned long& latestDist) {
  while (sensor.port.available()) {
    uint8_t c = sensor.port.read();
    
    for (int i = 0; i < 15; i++) {
      sensor.buffer[i] = sensor.buffer[i + 1];
    }
    sensor.buffer[15] = c;
    
    if (sensor.buffer[0] == 0x57 && sensor.buffer[1] == 0x00 && sensor.buffer[2] == 0xFF) {
      latestDist = sensor.buffer[8] | (sensor.buffer[9] << 8) | (sensor.buffer[10] << 16);
      sensor.buffer[0] = 0x00; // 清除帧头
    }
  }
}


void countEncoderL() {
  encoderTicksL++; 
}

void countEncoderR() {
  encoderTicksR++;
}