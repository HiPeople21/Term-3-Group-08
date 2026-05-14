uint8_t bufL[16];
uint8_t bufR[16];

void setup() {
  Serial.begin(115200);
  while (!Serial) { delay(10); }
  Serial1.begin(115200); // Left sensor
  Serial2.begin(115200); // Right sensor
}

void readSensor(HardwareSerial &port, uint8_t *buf, const char *label) {
  while (port.available()) {
    uint8_t c = port.read();
    for (int i = 0; i < 15; i++) buf[i] = buf[i + 1];
    buf[15] = c;
    if (buf[0] == 0x57 && buf[1] == 0x00 && buf[2] == 0xFF) {
      unsigned long dist = buf[8] | (buf[9] << 8) | (buf[10] << 16);
      Serial.print(label);
      Serial.print(": ");
      Serial.println(dist);
      buf[0] = 0x00;
    }
  }
}

void loop() {
  readSensor(Serial1, bufL, "LEFT");
  readSensor(Serial2, bufR, "RIGHT");
}
