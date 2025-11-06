#include "rover_logic.h"
#ifdef ARDUINO
  #include <Arduino.h>
#else
  // --- Minimal stand-ins for native testing ---
  #include <algorithm>
  #include <cmath>
  #define constrain(val,low,high) std::min(std::max((val),(low)),(high))
  inline long map(long x,long in_min,long in_max,long out_min,long out_max){
      return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
  }
#endif

int pulseToSpeed(int pulse){ if(pulse==0)return 0; int s=map(pulse,1000,2000,-255,255); return constrain(s,-255,255);}
int pulseToServoAngle(int pulse){ if(pulse==0)return 90; int a=map(pulse,1000,2000,0,180); return constrain(a,0,180);}
int applyDeadband(int v,int db){return(abs(v)<db)?0:v;}
void computeDriveSpeeds(int t,int s,int &L,int &R){int d=applyDeadband(pulseToSpeed(t));int st=applyDeadband(pulseToSpeed(s));L=constrain(d+st,-255,255);R=constrain(d-st,-255,255);}