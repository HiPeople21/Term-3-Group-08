#include "sensors.h"
#include <QTRSensors.h>

// --- IR Array ---
static QTRSensors qtr;
static const uint8_t kIRCount = 11;
static const uint8_t kIRPins[kIRCount] = {
  31, 30, 36,
  23, 28, 29, 24,
  37, 22, 33, 32};
static uint16_t irValues[kIRCount];

#define TRIG_FRONT 42
#define ECHO_FRONT 44

struct TOFSensor {
  Stream& port;
  const char* name;
  uint8_t buffer[16];
  unsigned long current_distance;
  unsigned long last_update_time;
};

static TOFSensor tofLeft  = {Serial1, "TOF-Left",  {0}, 0, 0};
static TOFSensor tofRight = {Serial4, "TOF-Right", {0}, 0, 0};

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
      sensor.current_distance = (unsigned long)sensor.buffer[8]
                              | ((unsigned long)sensor.buffer[9]  << 8)
                              | ((unsigned long)sensor.buffer[10] << 16);
      sensor.last_update_time = millis();
      sensor.buffer[0] = 0x00;
    }
  }
}

void readTOFSensors() {
  readOneTOF(tofLeft);
  readOneTOF(tofRight);
}

unsigned long getTOFLeftDist()  { return tofLeft.current_distance; }
unsigned long getTOFRightDist() { return tofRight.current_distance; }
bool isTOFLeftFresh()  { return (millis() - tofLeft.last_update_time) < 100; }
bool isTOFRightFresh() { return (millis() - tofRight.last_update_time) < 100; }

float getFrontDistanceCm() {
  digitalWrite(TRIG_FRONT, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_FRONT, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_FRONT, LOW);
  long duration = pulseIn(ECHO_FRONT, HIGH, 6000);
  if (duration == 0) return 999.0;
  return duration / 58.0;
}

void readUltrasonic() {
  static unsigned long lastRead = 0;
  if (millis() - lastRead < 100) return;
  lastRead = millis();
  getFrontDistanceCm();
}

void initIRArray() {
  qtr.setTypeRC();
  qtr.setSensorPins(kIRPins, kIRCount);

  delay(500);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);

  for (uint16_t i = 0; i < 400; i++) {
    qtr.calibrate();
  }

  digitalWrite(LED_BUILTIN, LOW);
  delay(1000);
}

void readIRArray() {
  static unsigned long lastRead = 0;
  if (millis() - lastRead < 100) return;
  lastRead = millis();
  qtr.readLineBlack(irValues);
}

uint16_t readIRPosition() {
  return qtr.readLineBlack(irValues);
}

uint16_t getIRValue(uint8_t index) {
  return irValues[index];
}

uint8_t getIRSensorCount() {
  return kIRCount;
}
