#ifndef MOTOR_H_INCLUDED
#define MOTOR_H_INCLUDED

#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#define LDIR 7
#define LPWM 6
#define RPWM 5
#define RDIR 4

/*
 *  motorL,motorRが正なら前進、負なら後退、0なら停止
 */
void motor_control2(int motorL, int motorR);
void motor_init();
#endif

