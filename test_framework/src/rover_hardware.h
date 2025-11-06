#ifndef ROVER_HARDWARE_H
#define ROVER_HARDWARE_H
void driveMotor(int pwmPin,int dirPin,int speed);
void driveAll(int left,int right);
#endif