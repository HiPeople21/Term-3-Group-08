#include <Wire.h>

void setup() {
  Serial.begin(9600);
  while (!Serial); // Wait for the Serial Monitor to open
  
  Serial.println("\n--- Giga R1 I2C Scanner ---");
  Serial.println("Scanning Wire1 (Pins 20/SDA & 21/SCL)...");
  
  Wire1.begin();
}

void loop() {
  byte error, address;
  int nDevices = 0;

  Serial.println("Scanning...");

  for (address = 1; address < 127; address++) {
    // Ping the address on Wire1
    Wire1.beginTransmission(address);
    error = Wire1.endTransmission();

    if (error == 0) {
      Serial.print("Device found at address 0x");
      if (address < 16) {
        Serial.print("0");
      }
      Serial.print(address, HEX);
      Serial.print("  (Decimal: ");
      Serial.print(address);
      Serial.println(")");
      nDevices++;
    } else if (error == 4) {
      Serial.print("Unknown error at address 0x");
      if (address < 16) {
        Serial.print("0");
      }
      Serial.println(address, HEX);
    }
  }

  if (nDevices == 0) {
    Serial.println("No I2C devices found.\n");
  } else {
    Serial.println("Scan complete.\n");
  }

  delay(5000); // Wait 5 seconds before scanning again
}