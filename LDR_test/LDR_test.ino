// LDR on Arduino Giga R1 WiFi
// Circuit: LDR between A0 and 3.3V, 10kΩ pull-down resistor to GND

#define LDR_PIN A7
const int LED_PIN    = LED_BUILTIN;

// Thresholds (tune these to your environment)
const int DARK_THRESHOLD   = 1000;   // below this = dark
const int BRIGHT_THRESHOLD = 3000;   // above this = bright

void setup() {
  Serial.begin(115200);
  while (!Serial);            // wait for Serial Monitor

  pinMode(LED_PIN, OUTPUT);
  analogReadResolution(12);   // Giga R1 supports 12-bit ADC (0–4095)

  Serial.println("LDR ready — 12-bit ADC (0-4095)");
}

void loop() {
  int raw = analogRead(LDR_PIN);

  // Convert to a 0–100 brightness percentage
  float brightness = (raw / 4095.0f) * 100.0f;

  // Simple light-level classification
  const char* level;
  if      (raw < DARK_THRESHOLD)   level = "DARK";
  else if (raw < BRIGHT_THRESHOLD) level = "DIM";
  else                             level = "BRIGHT";

  Serial.print("Raw: ");
  Serial.print(raw);
  Serial.print("  |  Brightness: ");
  Serial.print(brightness, 1);
  Serial.print("%  |  Level: ");
  Serial.println(level);

  // Turn on built-in LED when dark
  digitalWrite(LED_PIN, raw < DARK_THRESHOLD ? HIGH : LOW);

  delay(500);
}