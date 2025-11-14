#include <Servo.h>
#include \"rover_hardware.h\"

Servo arm1,arm2,storage;
String cmd=\"\";

void setup(){
  Serial.begin(115200);
  Serial.println(\"Type 'start' to begin rover hardware test...\");
  arm1.attach(A0); arm2.attach(A1); storage.attach(A2);
  pinMode(3,OUTPUT);pinMode(4,OUTPUT);pinMode(5,OUTPUT);pinMode(6,OUTPUT);
  pinMode(9,OUTPUT);pinMode(8,OUTPUT);pinMode(10,OUTPUT);pinMode(11,OUTPUT);
}

void loop(){
  if(Serial.available()) cmd=Serial.readStringUntil('\\n');
  if(cmd==\"start\"){
    Serial.println(\"-- Motor test --\");
    driveAll(150,150); delay(1000);
    driveAll(-150,-150); delay(1000);
    driveAll(0,0);
    Serial.println(\"-- Servo test --\");
    for(int i=0;i<=180;i+=15){arm1.write(i);arm2.write(180-i);storage.write(i);delay(100);}
    Serial.println(\"Test complete.\");
    cmd=\"\";
  }
}