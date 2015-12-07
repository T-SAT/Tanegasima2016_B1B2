#ifndef SERVO_H_INCLUDED
#define SERVO_H_INCLUDED

#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#define servo_CSPIN 3

/*move_servo
   説明:サーボを動かす
   出力値:なし
   入力値:なし
*/
void move_servo(void);

#endif

