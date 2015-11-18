#ifndef PRESSURE_H_INCLUDED
#define PRESSURE_H_INCLUDED

#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

extern const int LPS331AP_CS;
//const int SS = 10;      // 必ず 10 番を出力にすること
//const int MOSI = 11;
//const int MISO = 12;
//const int SCK  = 13;

extern const byte LPS331AP_ADDR;  // SA0 = GND
//const byte LPS331AP_ADDR = B1011101;// SA0 = VDD_IO

extern const byte LPS331AP_WHOAMI ;
extern const byte LPS331AP_CTRL1 ;
extern const byte LPS331AP_CTRL2 ;
extern const byte LPS331AP_CTRL3 ;
extern const byte LPS331AP_P_LL ;
extern const byte LPS331AP_P_L ;
extern const byte LPS331AP_P_H ;
extern const byte LPS331AP_T_L ;
extern const byte LPS331AP_T_H ;

extern const byte LPS331AP_RW ;
extern const byte LPS331AP_MS ;

void LPS331AP_init(void);
void LPS331AP_write(byte reg, byte val);
byte LPS331AP_read(byte reg);

#endif
