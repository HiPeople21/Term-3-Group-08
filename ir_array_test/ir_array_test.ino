const int testPin = 3; // We will test just one sensor first on Pin 3

void setup() {
  Serial.begin(9600);
  pinMode(LED_BUILTIN, OUTPUT);
  // Give the serial port a second to connect
  delay(2000); 
  Serial.println("Starting Custom Sensor Read...");
}

void loop() {
  // 1. Turn pin to OUTPUT and push it HIGH to charge the sensor's capacitor
  pinMode(testPin, OUTPUT);
  digitalWrite(testPin, HIGH);
  delayMicroseconds(15); // Wait 15 microseconds to fully charge

  // 2. Turn pin to INPUT and measure how fast it drains back to LOW
  pinMode(testPin, INPUT);
  unsigned long startTime = micros();
  unsigned long drainTime = 0;

  // 3. Count the microseconds until the pin drops to LOW
  while (digitalRead(testPin) == HIGH) {
    drainTime = micros() - startTime;
    
    // If it takes longer than 3000 microseconds, it's definitely on a black line.
    // We force a break here so Mbed OS never gets trapped and crashes!
    if (drainTime > 3000) {
      break; 
    }
  }

  // 4. Print the result
  Serial.print("Pin 3 Drain Time (us): ");
  Serial.println(drainTime);

  // 5. Briefly pause to let the GIGA's operating system breathe
  delay(100); 
}