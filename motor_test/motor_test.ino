#include <Wire.h>
#include "motors.h"

static const int trackSpeed = 800;

void setup() {
  Serial.begin(115200);
  while (!Serial && millis(
) < 3000);

  Wire1.begin();
  initMotors();

  Serial.println("[Motors] Ready.");
  Serial.println("W/S = Fwd/Rev | A/D = Left/Right | X = Stop");
}

void loop() {
  if (!Serial.available()) return;

  char cmd = Serial.read();
  if      (cmd == 'w' || cmd == 'W') { setRightTrack(trackSpeed);  setLeftTrack(trackSpeed);  Serial.println("Forward"); }
  else if (cmd == 's' || cmd == 'S') { setRightTrack(-trackSpeed); setLeftTrack(-trackSpeed); Serial.println("Reverse"); }
  else if (cmd == 'a' || cmd == 'A') { setRightTrack(trackSpeed);  setLeftTrack(-trackSpeed); Serial.println("Left"); }
  else if (cmd == 'd' || cmd == 'D') { setRightTrack(-trackSpeed); setLeftTrack(trackSpeed);  Serial.println("Right"); }
  else if (cmd == 'x' || cmd == 'X') { stopTracks();                                          Serial.println("Stop"); }
  else if (cmd == 'z' || cmd == 'Z') { playSequence();                                          Serial.println("Stop"); }
}

void playSequence() {
  for (int i = 0; i < 10; i++) {
    setRightTrack(trackSpeed);  setLeftTrack(trackSpeed);
    delay(1500);
    setRightTrack(-trackSpeed);  setLeftTrack(-trackSpeed);
    delay(1500);
    setRightTrack(trackSpeed);  setLeftTrack(-trackSpeed);
    delay(1500);
    setRightTrack(-trackSpeed);  setLeftTrack(trackSpeed);
    delay(1500);
  }
}
