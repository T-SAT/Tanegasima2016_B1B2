#ifndef FALL_H_INCLUDE
#define FALL_H_INCLUDE

#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#define ST_START  1
#define ST_UP     2
#define ST_TOP    3
#define ST_DOWN   4
#define ST_LAND   5

#define UP_ALTITUDE 25
#define DOWN_ALTITUDE 25
#define TOP_ALTITUDE 50
#define LAND_ALTITUDE 1

extern int cur_st;

int check_st(float dtitude);
void release_para(void);

#endif
