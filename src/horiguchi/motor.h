#ifndef MOTOR_H_INCLUDED
#define MOTOR_H_INCLUDED

#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#define LDir 4
#define LPWM 5
#define RPWM 6
#define RDir 7

/*
 *  motorL,motorRが正なら前進、負なら後退、0なら停止
 */
void motor_control(int motorL, int motorR);
void motor_init();
#endif

