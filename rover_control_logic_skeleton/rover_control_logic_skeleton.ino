#include <Servo.h>
// (plus controller library, e.g., PS2X_lib.h)


// --- Define pins for receiver inputs ---
const int chThrottlePin = 2;  // e.g. throttle (forward/reverse)
const int chSteeringPin = 3;  // e.g. left/right
const int chArm1Pin = 4;      // e.g. manipulator servo 1
const int chArm2Pin = 5;      // manipulator servo 2
const int chBoxPin = 6;       // storage box servo

//motor driver pins (example)
const int PWM_FL = 3, DIR_FL = 4;
const int PWM_FR = 5, DIR_FR = 6;
const int PWM_BL = 9, DIR_BL = 8;
const int PWM_BR = 10, DIR_BR = 11;

// servos
Servo armServo1;
Servo armServo2;
Servo storageServo;

// --- Helper to map RC pulse width to speed / servo angle ---
int pulseToSpeed(int pulse) {
  // Assuming 1000-2000 µs mapping to -255…+255
  int speed = map(pulse, 1000, 2000, -255, 255);
  speed = constrain(speed, -255, 255);
  return speed;
}
int pulseToServoAngle(int pulse) {
  // Map 1000-2000 µs to 0-180°
  int ang = map(pulse, 1000, 2000, 0, 180);
  ang = constrain(ang, 0, 180);
  return ang;
}

// Motor drive function
void driveMotor(int pwmPin, int dirPin, int speed) {
  if (speed >= 0) {
    digitalWrite(dirPin, HIGH);
    analogWrite(pwmPin, speed);
  } else {
    digitalWrite(dirPin, LOW);
    analogWrite(pwmPin, -speed);
  }
}
void driveAll(int leftSpeed, int rightSpeed) {
  driveMotor(PWM_FL, DIR_FL, leftSpeed);
  driveMotor(PWM_BL, DIR_BL, leftSpeed);
  driveMotor(PWM_FR, DIR_FR, rightSpeed);
  driveMotor(PWM_BR, DIR_BR, rightSpeed);
}

void setup() {
  Serial.begin(115200);
  // Pin modes for direction pins
  pinMode(DIR_FL, OUTPUT);
  pinMode(DIR_FR, OUTPUT);
  pinMode(DIR_BL, OUTPUT);
  pinMode(DIR_BR, OUTPUT);

  // Attach servos
  armServo1.attach( A0 );  // or another pin
  armServo2.attach( A1 );
  storageServo.attach( A2 );
}

void loop() {
  // Read pulse widths
  int throttlePulse = pulseIn(chThrottlePin, HIGH, 30000);
  int steerPulse = pulseIn(chSteeringPin, HIGH, 30000);
  int arm1Pulse = pulseIn(chArm1Pin, HIGH, 30000);
  int arm2Pulse = pulseIn(chArm2Pin, HIGH, 30000);
  int boxPulse = pulseIn(chBoxPin, HIGH, 30000);

  // Convert to control values
  int driveVal = pulseToSpeed(throttlePulse);
  int steerVal = pulseToSpeed(steerPulse);

  // For drive: use differential drive
  int leftSpeed = driveVal + steerVal;
  int rightSpeed = driveVal - steerVal;
  leftSpeed = constrain(leftSpeed, -255, 255);
  rightSpeed = constrain(rightSpeed, -255, 255);

  driveAll(leftSpeed, rightSpeed);

  // Manipulator control
  int a1Angle = pulseToServoAngle(arm1Pulse);
  int a2Angle = pulseToServoAngle(arm2Pulse);
  armServo1.write(a1Angle);
  armServo2.write(a2Angle);

  // Storage box
  int boxAngle = pulseToServoAngle(boxPulse);
  storageServo.write(boxAngle);

  // Debug output
  Serial.print("Thr: "); Serial.print(throttlePulse);
  Serial.print(" Steer: "); Serial.print(steerPulse);
  Serial.print(" Arm1: "); Serial.print(a1Angle);
  Serial.print(" Arm2: "); Serial.print(a2Angle);
  Serial.print(" Box: "); Serial.println(boxAngle);

  delay(20);
}