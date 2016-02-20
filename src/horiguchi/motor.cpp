#include "motor.h"


void motor_control(int motorL, int motorR)
{
  if (motorL == 0 && motorR == 0)
  {
    digitalWrite(LDir, LOW);
    analogWrite(LPWM, motorL);
    analogWrite(RPWM, motorR);
    digitalWrite(RDir, LOW);
  }

  else if (motorL <= 0 && motorR <= 0)
  {
    digitalWrite(LDir, HIGH);
    analogWrite(LPWM, abs(motorL));
    analogWrite(RPWM, abs(motorR));
    digitalWrite(RDir, HIGH);
  }
  else if (motorL <= 0 && motorR >= 0)
  {
    digitalWrite(LDir, HIGH);
    analogWrite(LPWM, abs(motorL));
    analogWrite(RPWM, motorR);
    digitalWrite(RDir, LOW);
  }
  else if (motorL >= 0 && motorR <= 0)
  {
    digitalWrite(LDir, LOW);
    analogWrite(LPWM, motorL);
    analogWrite(RPWM, abs(motorR));
    digitalWrite(RDir, HIGH);
  }
  else if (motorL >= 0 && motorR >= 0)
  {
    digitalWrite(LDir, LOW);
    analogWrite(LPWM, motorL);
    analogWrite(RPWM, motorR);
    digitalWrite(RDir, LOW);
  }
}

void motor_init()
{
  pinMode(LDir, OUTPUT);
  pinMode(LPWM, OUTPUT); //pwm
  pinMode(RPWM, OUTPUT);
  pinMode(RDir, OUTPUT); //pwm
}

