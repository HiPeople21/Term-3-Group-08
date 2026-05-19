#include <Wire.h>

void setup() {
  Serial.begin(9600);
  while (!Serial); // Wait for the Serial Monitor to open
  
  Serial.println("\n--- Giga R1 Dual I2C Scanner ---");
  
  // Initialize both I2C buses
  Wire.begin();   
  Wire1.begin();  
  Wire2.begin();
}

// Custom function to scan any I2C bus
void scanI2CBus(TwoWire &bus, const char* busName) {
  byte error, address;
  int nDevices = 0;

  Serial.print("Scanning ");
  Serial.print(busName);
  Serial.println("...");

  for (address = 1; address < 127; address++) {
    // Ping the address on the specified bus
    bus.beginTransmission(address);
    error = bus.endTransmission();

    if (error == 0) {
      Serial.print("  -> Device found at address 0x");
      if (address < 16) {
        Serial.print("0");
      }
      Serial.print(address, HEX);
      Serial.print("  (Decimal: ");
      Serial.print(address);
      Serial.println(")");
      nDevices++;
    } else if (error == 4) {
      Serial.print("  -> Unknown error at address 0x");
      if (address < 16) {
        Serial.print("0");
      }
      Serial.println(address, HEX);
    }
  }

  if (nDevices == 0) {
    Serial.println("  -> No I2C devices found.");
  } else {
    Serial.println("  -> Scan complete.");
  }
  Serial.println(); // Add a blank line for readability
}

void loop() {
  Serial.println("=====================================");
  
  // Note: On the GIGA R1, Pins 20 (SDA) and 21 (SCL) are usually the default 'Wire'
  scanI2CBus(Wire, "Wire");
  
  // Note: On the GIGA R1, 'Wire1' is usually mapped to D9 (SDA1) and D8 (SCL1)
  scanI2CBus(Wire1, "Wire1");

  scanI2CBus(Wire2, "Wire2");


  delay(5000); // Wait 5 seconds before scanning again
}