#include <SPI.h>
#include "pressure.h"
#include <math.h>

const int LPS331AP_CS = SS;
//const int SS = 10;      // 必ず 10 番を出力にすること
//const int MOSI = 11;
//const int MISO = 12;
//const int SCK  = 13;

const byte LPS331AP_ADDR = B1011100;  // SA0 = GND
//const byte LPS331AP_ADDR = B1011101;// SA0 = VDD_IO

const byte LPS331AP_WHOAMI = 0x0f;
const byte LPS331AP_CTRL1 = 0x20;
const byte LPS331AP_CTRL2 = 0x21;
const byte LPS331AP_CTRL3 = 0x22;
const byte LPS331AP_P_LL = 0x28;
const byte LPS331AP_P_L  = 0x29;
const byte LPS331AP_P_H  = 0x2A;
const byte LPS331AP_T_L  = 0x2B;
const byte LPS331AP_T_H  = 0x2C;

const byte LPS331AP_RW = 0x80;
const byte LPS331AP_MS = 0x40;

void LPS331AP_write(byte reg, byte val)
{
  digitalWrite(LPS331AP_CS, LOW);
  SPI.transfer(reg);
  SPI.transfer(val);
  digitalWrite(LPS331AP_CS, HIGH);
}

byte LPS331AP_read(byte reg)
{
  byte ret = 0;

  digitalWrite(LPS331AP_CS, LOW);
  SPI.transfer(reg | LPS331AP_RW);
  ret = SPI.transfer(0);
  digitalWrite(LPS331AP_CS, HIGH);
  
  return ret;
}

void LPS331AP_init(void){
 pinMode(SS, OUTPUT);
 digitalWrite(SS, HIGH);
 
 SPI.begin();
 SPI.setBitOrder(MSBFIRST);
 SPI.setClockDivider(SPI_CLOCK_DIV8); // 8MHz/8 = 1MHz; (max 10MHz)
  while (!Serial) {}
 
 Serial.println(LPS331AP_read(LPS331AP_WHOAMI), HEX); // should show BB
 
 LPS331AP_write(LPS331AP_CTRL1, B10010000);
 //   |||||||+ SPI Mode selection
 //   ||||||+- DELTA_EN
 //   |||||+-- BDU: block data update
 //   ||||+--- DIFF_EN: interrupt circuit enable
 //   |+++---- ODR2, ODR1, ODR0 (1Hz)
 //   +------- PD: 0: power down, 1: active
 }