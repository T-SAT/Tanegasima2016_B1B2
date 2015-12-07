#include <Servo.h>
#include "servo.h"

Servo myservo;

void move_servo()
{
  myservo.attach(servo_CSPIN, 800, 2300);

  myservo.writeMicroseconds(2300);
  delay(1500);
  myservo.writeMicroseconds(800);
  delay(1500);

/*
for(int i=800;i<2300;i++){
  myservo.writeMicroseconds(i);
  delay(1);
}
delay(1000);
for(int i=2300;i>800;i--){
  myservo.writeMicroseconds(i);
  delay(1);
}
delay(3000);
*/
}
