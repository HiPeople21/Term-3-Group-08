// --- Pin Definitions ---
// You can change these to match your specific wiring on the Giga R1 WiFi
const int buttonPin = 39;     // Connect the push button here
const int redPin = 38;        // Connect to the Red leg of the RGB LED
const int greenPin = 40;     // Connect to the Green leg of the RGB LED
// const int bluePin = 11;      // Connect to the Blue leg of the RGB LED

// --- State Variables ---
bool isBlinkingRed = false;  // false = Solid Green, true = Blinking Red
int buttonState = HIGH;      // Current reading from the input pin
int lastButtonState = HIGH;  // Previous reading from the input pin

// --- Debounce Variables ---
unsigned long lastDebounceTime = 0;  
const unsigned long debounceDelay = 50; // 50ms delay to filter out physical button bounce

// --- Blinking Variables ---
unsigned long previousMillis = 0;    
const long blinkInterval = 500;      // 500ms on, 500ms off
bool redLedState = LOW;              // Tracks if the red LED is currently on or off

void setup() {
  // Configure the LED pins as outputs
  pinMode(redPin, OUTPUT);
  pinMode(greenPin, OUTPUT);
  // pinMode(bluePin, OUTPUT);

  // Configure the button pin with the internal pull-up resistor.
  // This means the pin reads HIGH normally, and LOW when the button is pressed.
  pinMode(buttonPin, INPUT_PULLUP); 

  // Initialize the starting state (Solid Green)
  setLedSolidGreen();
}

void loop() {
  // 1. Read and Debounce the Button
  int reading = digitalRead(buttonPin);

  // Reset the debouncing timer if the button state changed (due to noise or a press)
  if (reading != lastButtonState) {
    lastDebounceTime = millis();
  }

  // Whatever the reading is at, it's been there for longer than the debounce delay
  if ((millis() - lastDebounceTime) > debounceDelay) {
    
    // If the button state has truly changed
    if (reading != buttonState) {
      buttonState = reading;

      // Only toggle the behavior when the button is pressed (transitions from HIGH to LOW)
      if (buttonState == LOW) {
        isBlinkingRed = !isBlinkingRed; // Toggle the state

        // Instantly update the LEDs based on the new state
        if (!isBlinkingRed) {
          setLedSolidGreen(); // Go back to green immediately
        } else {
          // Prepare for red blinking by turning off green and blue immediately
          digitalWrite(greenPin, LOW);
          // digitalWrite(bluePin, LOW);
          
          // Turn red ON immediately so the blink starts right away
          redLedState = HIGH; 
          digitalWrite(redPin, redLedState);
          previousMillis = millis(); // Reset the blink timer
        }
      }
    }
  }

  // Save the reading. Next time through the loop, it'll be the lastButtonState:
  lastButtonState = reading;

  // 2. Handle the Red LED Blinking (Non-Blocking)
  if (isBlinkingRed) {
    unsigned long currentMillis = millis();
    
    // Check if it's time to toggle the red LED
    if (currentMillis - previousMillis >= blinkInterval) {
      previousMillis = currentMillis;     // Save the last time we blinked the LED
      redLedState = !redLedState;         // Toggle the state (HIGH to LOW, or LOW to HIGH)
      digitalWrite(redPin, redLedState);  // Apply the state to the pin
    }
  }
}

// --- Helper Function ---
void setLedSolidGreen() {
  digitalWrite(redPin, LOW);
  digitalWrite(greenPin, HIGH);
  // digitalWrite(bluePin, LOW);
}