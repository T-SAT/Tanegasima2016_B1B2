#include <SPI.h>
#include <SoftwareSerial.h>
#include "accel.h"
#include "fall.h"
#include "skLPSxxSPI.h"
#include "sonic.h"
#include "wireless.h"

#define CSPin 2
/*
  skLPSxxx LPS(LPS25H, CSPin);

  float pressure_origin;

  void setup() {
  // put your setup code here, to run once:
  LPS.PressureRead();
  pressure_origin = LPS.getPressure();

  }

  void loop() {
  float altitude;
  float pressure_cur;
  int state = ST_START;
  float transData[2];



  while (state != ST_LAND) {
    LPS.PressureRead();
    pressure_cur = LPS.getPressure();
    altitude = LPS.AltitudeCalc(pressure_origin, pressure_cur);
    transData[0] = pressure_cur;
    transData[1] = altitude;
    transferData(transData, 2);
    state = check_st(altitude);
  }

  release_para();

  // put your main code here, to run repeatedly:

  }
*/
void setup() {
  Serial.begin(9600);
  wireless_init();
  sonic_init();
}
void loop() {
  float x[1];
  x[0] = measure_sonic();

  transferData(x, 1);
}

