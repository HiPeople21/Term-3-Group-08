#include "sensors.h"

#define TRIG_FRONT 44
#define ECHO_FRONT 42

struct TOFSensor {
  Stream& port;
  const char* name;
  uint8_t buffer[16];
};

static TOFSensor tofLeft = {Serial1, "TOF-Left", {0}};
static TOFSensor tofRight  = {Serial4, "TOF-Right",  {0}};

void initSensors() {
  pinMode(TRIG_FRONT, OUTPUT);
  pinMode(ECHO_FRONT, INPUT);
  Serial1.begin(921600);
  Serial4.begin(921600);
}

static void readOneTOF(TOFSensor& sensor) {
  while (sensor.port.available()) {
    uint8_t c = sensor.port.read();
    for (int i = 0; i < 15; i++) sensor.buffer[i] = sensor.buffer[i + 1];
    sensor.buffer[15] = c;
    if (sensor.buffer[0] == 0x57 && sensor.buffer[1] == 0x00 && sensor.buffer[2] == 0xFF) {
      unsigned long dist = (unsigned long)sensor.buffer[8]
                         | ((unsigned long)sensor.buffer[9]  << 8)
                         | ((unsigned long)sensor.buffer[10] << 16);
      Serial.print(sensor.name);
      Serial.print(": ");
      Serial.print(dist);
      Serial.println(" mm");
      sensor.buffer[0] = 0x00;
    }
  }
}

void readTOFSensors() {
  readOneTOF(tofLeft);
  readOneTOF(tofRight);
}

void readUltrasonic() {
  static unsigned long lastRead = 0;
  if (millis() - lastRead < 100) return;
  lastRead = millis();

  digitalWrite(TRIG_FRONT, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_FRONT, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_FRONT, LOW);
  long duration = pulseIn(ECHO_FRONT, HIGH, 30000UL);
  float distCm = duration / 58.0f;
  Serial.print("Ultrasonic: ");
  Serial.print(distCm);
  Serial.println(" cm");
}
