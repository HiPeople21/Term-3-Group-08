// uint8_t bufL[16];
// uint8_t bufR[16];

// void setup() {
//   Serial.begin(115200);
//   while (!Serial) { delay(10); }
//   Serial1.begin(115200); // Left sensor
//   Serial4.begin(115200); // Right sensor
// }

// void readSensor(HardwareSerial &port, uint8_t *buf, const char *label) {
//   while (port.available()) {
//     uint8_t c = port.read();
//     for (int i = 0; i < 15; i++) buf[i] = buf[i + 1];
//     buf[15] = c;
//     if (buf[0] == 0x57 && buf[1] == 0x00 && buf[2] == 0xFF) {
//       unsigned long dist = buf[8] | (buf[9] << 8) | (buf[10] << 16);
//       Serial.print(label);
//       Serial.print(": ");
//       Serial.println(dist);
//       buf[0] = 0x00;
//     }
//   }
// }

// void loop() {
//   readSensor(Serial1, bufL, "LEFT");
//   readSensor(Serial4, bufR, "RIGHT");
// }


// #include "TOF_Sense.h"

// void setup() {
//   Serial.begin(115200);//Initialize the USB serial port baud rate to 115200 初始化USB串口波特率到115200
//   TOF_UART.begin(115200);//Initialize the TOF serial port baud rate to 115200 初始化TOF串口波特率到115200
// }

// void loop() {
//   TOF_Active_Decoding();//Query and decode TOF data 查询获取TOF数据，并进行解码
//   // TOF_Inquire_Decoding(0);//Query and decode TOF data 查询获取TOF数据，并进行解码
//   delay(20);//The refresh rate defaults to 50HZ. If the refresh rate is set to 100HZ, the time here is 1/100=0.01 刷新率默认为50HZ,如果刷新率设置成100HZ,则这里的时间为1/100=0.01s
// }


// -------------------------------------------------------------
// Arduino GIGA R1 WiFi - Dual Waveshare ToF Sensor (UART)
// Sensor 1: TX0 / RX0 -> Serial1
// Sensor 2: TX3 / RX3 -> Serial4
// -------------------------------------------------------------
// -------------------------------------------------------------
// Arduino GIGA R1 WiFi - Dual Waveshare ToF Sensor (UART)
// Sensor 1: TX0 / RX0 -> Serial1
// Sensor 2: TX3 / RX3 -> Serial4
// -------------------------------------------------------------

// const long SENSOR_BAUD = 921600; // Waveshare TOF Mini factory default

// // A structure to keep track of the data frames for each sensor
// struct TOFSensor {
//   Stream& port;
//   const char* name;
//   uint8_t buffer[16];
//   uint8_t index;
// };

// // Initialize our two sensors
// TOFSensor sensor1 = {Serial1, "Sensor 1 (TX0/RX0)", {0}, 0};
// TOFSensor sensor2 = {Serial4, "Sensor 2 (TX3/RX3)", {0}, 0};

// void setup() {
//   Serial.begin(115200);
//   while (!Serial && millis() < 3000); 

//   Serial1.begin(SENSOR_BAUD); 
//   Serial4.begin(SENSOR_BAUD);
  
//   Serial.println("[SYSTEM] Dual Waveshare ToF Sensors Active.");
//   Serial.println("[SYSTEM] Reading actual distance values in mm...\n");
// }

// void loop() {
//   // Continuously check both sensors
//   readTOFSensor(sensor1);
//   readTOFSensor(sensor2);
// }

// // Smart parser that extracts the real distance from the hex packet
// void readTOFSensor(TOFSensor& sensor) {
  
//   while (sensor.port.available() > 0) {
//     uint8_t incomingByte = sensor.port.read();

//     // 1. Look for the Frame Header (always 0x57)
//     if (sensor.index == 0 && incomingByte != 0x57) {
//       continue; // Ignore garbage data until we see the header
//     }
    
//     // 2. Look for the Function Mark (always 0x00 for distance data)
//     if (sensor.index == 1 && incomingByte != 0x00) {
//       sensor.index = 0; // Reset if it's the wrong packet type
//       continue;
//     }

//     // 3. Store the byte in our buffer
//     sensor.buffer[sensor.index++] = incomingByte;

//     // 4. Once we have collected a full 16-byte frame...
//     if (sensor.index == 16) {
//       sensor.index = 0; // Reset the index for the next incoming packet

//       // 5. Verify the Checksum (Sum of bytes 0-14 should equal byte 15)
//       uint8_t sum = 0;
//       for (int i = 0; i < 15; i++) {
//         sum += sensor.buffer[i];
//       }

//       if (sum == sensor.buffer[15]) {
//         // --- PACKET IS VALID: DECODE THE MATH ---
        
//         // Distance is stored in little-endian format across bytes 8, 9, and 10
//         uint32_t distance_mm = sensor.buffer[8] | (sensor.buffer[9] << 8) | (sensor.buffer[10] << 16);
        
//         // Status is stored in byte 11 (0 means valid distance, anything else is an error code)
//         uint8_t status = sensor.buffer[11];

//         Serial.print(sensor.name);
        
//         if (status == 0) {
//           Serial.print(" | Distance: ");
//           Serial.print(distance_mm);
//           Serial.println(" mm");
//         } else {
//           // If the sensor is pointing at the sky or too close to an object
//           Serial.print(" | Out of range / Error Code: ");
//           Serial.println(status);
//         }
//       }
//     }
//   }
// }

// -------------------------------------------------------------
// Arduino GIGA R1 WiFi - Dual Waveshare ToF Sensor (UART)
// Sensor 1: TX0 / RX0 -> Serial1
// Sensor 2: TX3 / RX3 -> Serial4
// -------------------------------------------------------------

// A structure to hold the sliding window buffer for each sensor separately
struct TOFSensor {
  Stream& port;
  const char* name;
  uint8_t buffer[16];
};

// Initialize our two sensors with their own empty buffers
TOFSensor sensor1 = {Serial1, "Sensor 1 (TX0)", {0}};
TOFSensor sensor2 = {Serial4, "Sensor 2 (TX3)", {0}};

void setup() {
  Serial.begin(115200); 
  while (!Serial && millis() < 3000); 
  
  Serial.println("--- Dual Waveshare TOF Decoder Started ---");
  
  // 921600 is the factory default for the sensor. 
  Serial1.begin(921600); 
  Serial4.begin(921600); 
}

void loop() {
  // Check both sensors as fast as possible
  readTOFSensor(sensor1);
  readTOFSensor(sensor2);
}

// Your exact sliding-window logic, made reusable!
void readTOFSensor(TOFSensor& sensor) {
  
  while (sensor.port.available()) {
    uint8_t c = sensor.port.read();
    
    // Shift the buffer left by 1 to make room for the new byte
    for (int i = 0; i < 15; i++) {
      sensor.buffer[i] = sensor.buffer[i + 1];
    }
    
    // Put the newest byte at the end of the array
    sensor.buffer[15] = c;
    
    // The Waveshare data frame ALWAYS starts with these 3 specific bytes
    if (sensor.buffer[0] == 0x57 && sensor.buffer[1] == 0x00 && sensor.buffer[2] == 0xFF) {
      
      // The distance in mm is split across bytes 8, 9, and 10. 
      // We shift and combine them into a single number.
      unsigned long distance_mm = sensor.buffer[8] | (sensor.buffer[9] << 8) | (sensor.buffer[10] << 16);
      
      // Print the result with the sensor's name
      Serial.print(sensor.name);
      Serial.print(" | Distance: ");
      Serial.print(distance_mm);
      Serial.println(" mm");
      
      // Wipe the start byte so we don't accidentally read this same frame twice
      sensor.buffer[0] = 0x00;
    }
  }
}