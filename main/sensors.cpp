#include "sensors.h"
#include <QTRSensors.h>

// --- IR Array ---
static QTRSensors qtr;
static const uint8_t kIRCount = 12;
static const uint8_t kIRPins[kIRCount] = {31, 30, 27, 36, 23, 28, 29, 24, 37, 22, 33, 32};
static uint16_t irValues[kIRCount];

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

void initIRArray() {
  qtr.setTypeRC();
  qtr.setSensorPins(kIRPins, kIRCount);

  delay(500);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
  Serial.println("[IR] Calibrating — sweep all 12 sensors across the line...");

  for (uint16_t i = 0; i < 400; i++) {
    qtr.calibrate();
  }

  digitalWrite(LED_BUILTIN, LOW);
  Serial.println("[IR] Calibration complete.");

  Serial.print("[IR] MIN: ");
  for (uint8_t i = 0; i < kIRCount; i++) {
    Serial.print(qtr.calibrationOn.minimum[i]);
    Serial.print(' ');
  }
  Serial.println();

  Serial.print("[IR] MAX: ");
  for (uint8_t i = 0; i < kIRCount; i++) {
    Serial.print(qtr.calibrationOn.maximum[i]);
    Serial.print(' ');
  }
  Serial.println();

  delay(1000);
}

void readIRArray() {
  static unsigned long lastRead = 0;
  if (millis() - lastRead < 100) return;
  lastRead = millis();

  uint16_t position = qtr.readLineBlack(irValues);

  for (uint8_t i = 0; i < kIRCount; i++) {
    Serial.print(irValues[i]);
    Serial.print('\t');
  }

  int err = 6000 - (int)position;
  if (err < 0) err = -err;
  Serial.print("| Pos: ");
  Serial.print(position);
  Serial.print(" | Err: ");
  Serial.println(err);
}
