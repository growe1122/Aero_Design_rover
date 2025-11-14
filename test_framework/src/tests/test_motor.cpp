#include <Arduino.h>

// Set to your MD20A wiring
const int PWM_PIN = 5;  // must be PWM-capable
const int DIR_PIN = 6;

void setup() {
  Serial.begin(115200);
  pinMode(PWM_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  Serial.println("=== Motor Driver Test (CPP) ===");
}

void ramp(bool forward) {
  digitalWrite(DIR_PIN, forward ? HIGH : LOW);
  for (int s = 0; s <= 255; s += 5) {
    analogWrite(PWM_PIN, s);
    delay(15);
  }
  delay(300);
  for (int s = 255; s >= 0; s -= 5) {
    analogWrite(PWM_PIN, s);
    delay(10);
  }
  delay(300);
}

void loop() {
  ramp(true);
  ramp(false);
}
