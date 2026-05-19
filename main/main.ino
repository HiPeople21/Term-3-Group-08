// #include <Wire.h>
// #include <MFRC522_I2C.h>
// #include <Motoron.h>

// // --- HARDWARE SETUP ---

// // 1. RFID Scanner (M5Stack WS1850S) on Wire1
// MFRC522_I2C mfrc522(0x28, -1, &Wire1); 

// // 2. Single Motor Shield (Assuming the one soldered to Address 0x12)
// MotoronI2C mc; 

// // 3. Encoder Pins for the Planter
// #define ENCODER_A 22
// #define ENCODER_B 24

// // --- VARIABLES & SETTINGS ---

// // Track Settings
// int trackSpeed = 800; // Speed for driving the tracks via WASD

// // Planter Settings
// volatile long encoderPos = 0; 
// long targetPos = 0;
// float countsPerRevolution = 1400.0; 
// long ticksFor60Degrees = countsPerRevolution / 6; // 250 ticks
// int planterSpeed = 600; // Linear speed for the planter rotation


// // --- TRACK HELPER FUNCTIONS ---

// void setRightTrack(int speed) {
//   // Motor 1 drives the Right Track
//   mc.setSpeed(1, -speed);
// }

// void setLeftTrack(int speed) {
//   // Motor 3 drives the Left Track 
//   // *NOTE: If the left track drives backward when moving forward, change this to -speed!
//   mc.setSpeed(3, -speed);
// }

// void stopTracks() {
//   mc.setSpeed(1, 0);
//   mc.setSpeed(3, 0);
// }


// // --- MAIN SETUP ---

// void setup() {
//   Serial.begin(9600);
//   while (!Serial);

//   // 1. Initialize the I2C Bus
//   Wire1.begin();

//   // 2. Initialize the RFID Reader
//   mfrc522.PCD_Init();

//   // // 3. Initialize the Motor Shield
//   // mc.setBus(&Wire1);
//   // mc.setAddress(18); // Decimal 18 is Hex 0x12
//   // mc.reinitialize();
//   // mc.clearResetFlag();

//   // // 4. Initialize the Planter Encoder
//   // pinMode(ENCODER_A, INPUT_PULLUP);
//   // pinMode(ENCODER_B, INPUT_PULLUP);
//   // attachInterrupt(digitalPinToInterrupt(ENCODER_A), updateEncoder, CHANGE);

//   Serial.println("--- SINGLE-SHIELD MASTER ONLINE ---");
//   Serial.println("Tracks: W = Fwd | S = Rev | A = Left | D = Right | X = Stop");
//   Serial.println("RFID Scanner: Active.");
//   Serial.println("--------------------------------------------------");
// }


// // --- MAIN LOOP ---

// void loop() {
  
//   // ==========================================
//   // PART 1: MANUAL TRACK CONTROLS
//   // ==========================================
//   // if (Serial.available() > 0) {
//   //   char cmd = Serial.read();

//   //   if (cmd == 'w' || cmd == 'W') {
//   //     setRightTrack(trackSpeed);
//   //     setLeftTrack(trackSpeed);
//   //   } 
//   //   else if (cmd == 's' || cmd == 'S') {
//   //     setRightTrack(-trackSpeed);
//   //     setLeftTrack(-trackSpeed);
//   //   } 
//   //   else if (cmd == 'a' || cmd == 'A') {
//   //     setRightTrack(trackSpeed);
//   //     setLeftTrack(-trackSpeed);
//   //   } 
//   //   else if (cmd == 'd' || cmd == 'D') {
//   //     setRightTrack(-trackSpeed);
//   //     setLeftTrack(trackSpeed);
//   //   } 
//   //   else if (cmd == 'x' || cmd == 'X') {
//   //     stopTracks();
//   //   }
//   // }

//   // ==========================================
//   // PART 2: AUTOMATED RFID PLANTER SEQUENCE
//   // ==========================================
//   if (mfrc522.PICC_IsNewCardPresent() && mfrc522.PICC_ReadCardSerial()) {
    
//     // 1. Stop the tracks for safety while the planter moves
//     // stopTracks(); 
//     Serial.println("\n[RFID SCANNED] Tag Accepted. Moving Planter (M2) 60 Degrees...");

//     // 2. Set the new target position for the planter (+250 ticks)
//     targetPos += ticksFor60Degrees;

//     // 3. Drive the Planter Motor (M2) until it reaches the target
//     while (abs(targetPos - encoderPos) > 5) {
      
//       long error = targetPos - encoderPos;
//       int currentSpeed = (error > 0) ? planterSpeed : -planterSpeed;
      
//       // Send speed to Motor 2 (The Planter Motor)
//       // mc.setSpeed(2, currentSpeed); 
      
//       // Small delay to prevent spamming the I2C wires too fast
//       delay(5); 
//     }

//     // 4. Target Reached! Stop the planter motor
//     // mc.setSpeed(2, 0); 
//     Serial.println("[PLANTER] Target Reached. Returning to manual control.");
//     Serial.println("--------------------------------------------------");

//     // 5. Halt the RFID tag so it doesn't trigger repeatedly 
//     // mfrc522.PICC_HaltA(); 
//   }
// }

// // --- ENCODER INTERRUPT ---
// void updateEncoder() {
//   if (digitalRead(ENCODER_A) == digitalRead(ENCODER_B)) {
//     encoderPos++; 
//   } else {
//     encoderPos--; 
//   }
// }
