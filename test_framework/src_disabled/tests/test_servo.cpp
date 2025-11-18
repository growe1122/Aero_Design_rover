#include <Arduino.h>
#include <Servo.h>

Servo testServo;

void setup() {
  Serial.begin(115200);
  testServo.attach(A0);   // change if you’ll use another pin
  Serial.println("=== Servo Test (CPP) ===");
  Serial.println("Type an angle 0–180 and press Enter");
}

void loop() {
  if (Serial.available()) {
    int angle = Serial.parseInt();
    if (angle >= 0 && angle <= 180) {
      testServo.write(angle);
      Serial.print("Moved to: ");
      Serial.println(angle);
    }
  }
}
