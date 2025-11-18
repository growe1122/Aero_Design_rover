#ifndef ROVER_LOGIC_H
#define ROVER_LOGIC_H
int pulseToSpeed(int pulse);
int pulseToServoAngle(int pulse);
int applyDeadband(int val, int db = 20);
void computeDriveSpeeds(int throttlePulse, int steerPulse, int &leftSpeed, int &rightSpeed);
#endif