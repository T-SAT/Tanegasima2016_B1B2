#ifndef WIRELESS_H_INCLUDED
#define WIRELESS_H_INCLUDED

#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#define SS_RX 10 //ソフトウェアシリアルrxピン
#define SS_TX 12 //ソフトウェアシリアルtxピン

void transferData(float data[], int num);
void wireless_init();
#endif
