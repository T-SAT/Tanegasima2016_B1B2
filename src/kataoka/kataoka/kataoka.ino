#include <SPI.h>
#include <SoftwareSerial.h>
#include "accel.h"
#include "fall.h"
#include "skLPSxxSPI.h"
#include "sonic.h"
#include "wireless.h"

#define CSPin 9
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
void ReceiveStr(char *str){
  int i;
  char ch;
  
  for(i = 0; ch != '!';) {
    if(wirelessAvailable()) {
      ch = receiveData();
      str[i] = ch;
      i++;
    }
  }
  str[i-1] = '\0';
}

void setup() {
  char str[100];

  memset(str, 0, sizeof(str));
  Serial.begin(9600);
  SPI.begin();

  wireless_init();
  // sonic_init();
  init_accel(7);

  TransferStr("Ready...");
  while(strcmp(str, "START") != 0) {
     ReceiveStr(str);
  }

  TransferStr("Start!");
}

void loop() {
  int i;
  float x, y, z;
  float s[3];
  char ch;

  measure_accel(&x, &y, &z);

/*
  Serial.print(x);    // X axis (deg/sec)
  Serial.print("\t");
  Serial.print(y);    // Y axis (deg/sec)
  Serial.print("\t");
  Serial.println(z);  // Z axis (deg/sec)
*/
  s[0] = x;
  s[1] = y;
  s[2] = z;

  //transferData(s, 3);

  delay(10);

}

