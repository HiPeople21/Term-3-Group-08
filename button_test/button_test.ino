const int buttonPin1 = 2;
const int buttonPin2 = 3;

void setup() {
  Serial.begin(115200);

  pinMode(buttonPin1, INPUT_PULLUP);
  pinMode(buttonPin2, INPUT_PULLUP);
}

void loop()
{
  bool buttonReading1 = digitalRead(buttonPin1);
  bool buttonReading2 = digitalRead(buttonPin2);
  if (buttonReading1 == LOW) {
  Serial.print("Button 1: ");
  Serial.println(buttonReading1);}
  if (buttonReading2 == LOW) {
  Serial.print("Button 2: ");
  Serial.println(buttonReading2);}
  delay(100);
}