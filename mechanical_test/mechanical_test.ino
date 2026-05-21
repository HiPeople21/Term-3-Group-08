// Define LED pins
const int redLedPin = 38;
const int greenLedPin = 40;

// Define Button pins
// const int button1Pin = 46;
const int button2Pin = 48;

void setup() {
  // Configure LED pins as outputs
  pinMode(redLedPin, OUTPUT);
  pinMode(greenLedPin, OUTPUT);

  // Configure Button pins as inputs with internal pull-up resistors
  // Wiring: Connect the button between the pin and Ground (GND).
  pinMode(button1Pin, INPUT_PULLUP);
  pinMode(button2Pin, INPUT_PULLUP);
}

void loop() {
  // Read the current state of both buttons
  int button1State = digitalRead(button1Pin);
  int button2State = digitalRead(button2Pin);

  // Check if EITHER button is pressed. 
  // Because we use INPUT_PULLUP, a pressed button reads LOW.
  if (button1State == LOW || button2State == LOW) {
    // Button is pressed: Turn Green ON, Red OFF
    digitalWrite(redLedPin, LOW);
    digitalWrite(greenLedPin, HIGH);
  } else {
    // Normally (no buttons pressed): Turn Red ON, Green OFF
    digitalWrite(redLedPin, HIGH);
    digitalWrite(greenLedPin, LOW);
  }
}