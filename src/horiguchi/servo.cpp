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

}
