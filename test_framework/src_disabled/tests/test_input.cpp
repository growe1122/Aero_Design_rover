#include <Arduino.h>

// RC receiver channels
const int ch1 = 2;
const int ch2 = 3;
const int ch3 = 4;

void setup() {
  Serial.begin(115200);
  pinMode(ch1, INPUT);
  pinMode(ch2, INPUT);
  pinMode(ch3, INPUT);
  Serial.println("=== RC Input Test (CPP) ===");
}

void loop() {
  int p1 = pulseIn(ch1, HIGH, 30000);
  int p2 = pulseIn(ch2, HIGH, 30000);
  int p3 = pulseIn(ch3, HIGH, 30000);

  Serial.print("CH1: "); Serial.print(p1);
  Serial.print("  CH2: "); Serial.print(p2);
  Serial.print("  CH3: "); Serial.println(p3);

  delay(100);
}
