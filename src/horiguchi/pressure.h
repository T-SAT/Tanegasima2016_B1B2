#ifndef SONIC_H_INCLUDED
#define SONIC_H_INCLUDED

#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

extern const int LPS331AP_CS = SS;
//const int SS = 10;      // 必ず 10 番を出力にすること
//const int MOSI = 11;
//const int MISO = 12;
//const int SCK  = 13;

extern const byte LPS331AP_ADDR = B1011100;  // SA0 = GND
//const byte LPS331AP_ADDR = B1011101;// SA0 = VDD_IO

extern const byte LPS331AP_WHOAMI = 0x0f;
extern const byte LPS331AP_CTRL1 = 0x20;
extern const byte LPS331AP_CTRL2 = 0x21;
extern const byte LPS331AP_CTRL3 = 0x22;
extern const byte LPS331AP_P_LL = 0x28;
extern const byte LPS331AP_P_L  = 0x29;
extern const byte LPS331AP_P_H  = 0x2A;
extern const byte LPS331AP_T_L  = 0x2B;
extern const byte LPS331AP_T_H  = 0x2C;

extern const byte LPS331AP_RW = 0x80;
extern const byte LPS331AP_MS = 0x40;

void LPS331AP_write(byte reg, byte val);
byte LPS331AP_read(byte reg);

#endif
