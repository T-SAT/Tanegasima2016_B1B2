#ifndef ACCEL_H_INCLUDE
#define ACCEL_H_INCLUDE

#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

//Assign the Chip Select signal to pin 10.
extern int CS2;
extern char POWER_CTL;	//Power Control Register
extern char DATA_FORMAT;
extern char DATAX0;	//X-Axis Data 0
extern char DATAX1;	//X-Axis Data 1
extern char DATAY0;	//Y-Axis Data 0
extern char DATAY1;	//Y-Axis Data 1
extern char DATAZ0;	//Z-Axis Data 0
extern char DATAZ1;	//Z-Axis Data 1

/////////////ユーザ用関数///////////////////
void measure_accel(float *x, float *y, float *z);
////////////////////////////////////////////

void init_accel(int CS2);
void writeRegister(char registerAddress, char value);
void readRegister(char registerAddress, int numBytes, char * values);

#endif
