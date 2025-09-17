#include "SoftwareSerial.h"
#include "Servo.h"
SoftwareSerial myserial(0,1) ;
Servo RIGHT_ARM;
Servo LEFT_ARM ;
Servo HEAD;
Servo BODY;
void setup() {
  myserial.begin(9600);
  RIGHT_ARM.attach(22);
  LEFT_ARM.attach(23);
  HEAD.attach(24);
  BODY.attach(25);
  HEAD.write(90);
  BODY.write(90);

}

void loop() {
  if(myserial.read() == 'shake hands' ){
   RIGHT_ARM.write(90);
   LEFT_ARM.write(90);
   HEAD.write(180);
   delay(500);
   HEAD.write(90);
   delay(500);
   HEAD.write(0);
   BODY.write(180);
   delay(500);
   BODY.write(90);
   delay(500);
   BODY.write(0);



  }

}
