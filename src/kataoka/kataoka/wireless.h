#ifndef WIRELESS_H_INCLUDED
#define WIRELESS_H_INCLUDED

#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#define SS_RX 1 //ソフトウェアシリアルrxピン
#define SS_TX 2 //ソフトウェアシリアルtxピン

void transferData(float data[], int num);

#endif
