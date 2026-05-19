#define TRIG_FRONT 44
#define ECHO_FRONT 42

void setup() {
  pinMode(TRIG_FRONT, OUTPUT);
  pinMode(ECHO_FRONT, INPUT);
  Serial.begin(9600);
  Serial.println("HC-SR04 Test Starting...");
}

float getDistance(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  long duration = pulseIn(echoPin, HIGH);
  return duration / 58.0;
}

void loop() {
  float front = getDistance(TRIG_FRONT, ECHO_FRONT);
  // Serial.print("Front distance: ");
  Serial.println(front);
  // Serial.println(" cm");
  delay(60);
}