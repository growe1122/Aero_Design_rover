#include "rover_hardware.h"
#include <Arduino.h>
void driveMotor(int pwmPin,int dirPin,int speed){
  digitalWrite(dirPin,(speed>=0)?HIGH:LOW);
  analogWrite(pwmPin,abs(speed));
}
void driveAll(int L,int R){
  driveMotor(3,4,L); driveMotor(9,8,L);
  driveMotor(5,6,R); driveMotor(10,11,R);
}