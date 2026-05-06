uint8_t buffer[16];

void setup() {
  Serial.begin(115200); 
  while (!Serial) { delay(10); }
  
  Serial.println("--- Waveshare TOF Decoder Started ---");
  
  // 921600 is the factory default for the sensor. 
  // (If it says "Started" but prints nothing else, change this to 115200)
  Serial1.begin(921600); 
}

void loop() {
  while (Serial1.available()) {
    uint8_t c = Serial1.read();
    
    // Shift the buffer left by 1 to make room for the new byte
    for (int i = 0; i < 15; i++) {
      buffer[i] = buffer[i + 1];
    }
    // Put the newest byte at the end of the array
    buffer[15] = c;
    
    // The Waveshare data frame ALWAYS starts with these 3 specific bytes
    if (buffer[0] == 0x57 && buffer[1] == 0x00 && buffer[2] == 0xFF) {
      
      // The distance in mm is split across bytes 8, 9, and 10. 
      // We shift and combine them into a single number.
      unsigned long distance_mm = buffer[8] | (buffer[9] << 8) | (buffer[10] << 16);
      
      Serial.print("Distance: ");
      Serial.print(distance_mm);
      Serial.println(" mm");
      
      // Wipe the start byte so we don't accidentally read this same frame twice
      buffer[0] = 0x00;
    }
  }
}