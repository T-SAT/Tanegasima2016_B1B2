#ifndef INFRARED_H_INCLUDED
#define INFRARED_H_INCLUDED

#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

/*
*measure_infrared
*説明:
*出力値:型、値の範囲、単位
*入力値:型、引数名、値の範囲、単位
*/
float measure_infrared(void);


#endif
