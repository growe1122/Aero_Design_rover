#include <Servo.h>
#include \"rover_logic.h\"
#include \"rover_hardware.h\"

#define USE_EASED_SERVO

Servo arm1,arm2,storage;
int arm1Pos=90,arm2Pos=90,storagePos=90;

int smoothEaseMove(Servo &s,int cur,int tgt,float k=0.15){
#ifdef USE_EASED_SERVO
  float d=tgt-cur; cur+=d*k; s.write((int)cur);
#else
  cur=tgt; s.write(cur);
#endif
  return cur;
}

void setup(){
  Serial.begin(115200);
  arm1.attach(A0); arm2.attach(A1); storage.attach(A2);
  pinMode(3,OUTPUT);pinMode(4,OUTPUT);pinMode(5,OUTPUT);pinMode(6,OUTPUT);
  pinMode(9,OUTPUT);pinMode(8,OUTPUT);pinMode(10,OUTPUT);pinMode(11,OUTPUT);
}

void loop(){
  // Example logic placeholder
}