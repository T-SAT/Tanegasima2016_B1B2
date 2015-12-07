#include <SPI.h>
#include <SD.h>
#include <Servo.h>
#include "pressure.h"
#include "sonic.h"
#include "skLPSxxSPI.h"
#include "sd.h"
#include "servo.h"

<<<<<<< HEAD

skLPSxxx LPS(LPS25H, SS);
=======
skLPSxxx LPS(LPS25H, 2);  //圧力センサの型番の設定
>>>>>>> c20859c890707a590068970930fdcfd26d90867c

float pressure_origin;

void setup() {
  Serial.begin(9600);
  pinMode(10, OUTPUT);

  /////////////SD設定部///////////////
  SD.begin(SD_CSPIN);
  //////////////超音波センサ設定部///////////////////
  sonic_init();

  ////////////圧力センサ設定部////////////////
  LPS.PressureInit() ;
  LPS.PressureRead();
  pressure_origin = LPS.getPressure();

}

void loop() {
  move_servo();
  ///////////////圧力センサ//////////////////
  float h;
  float t;
  float p;
  float pressure[3];

  LPS.PressureRead();
  t = LPS.getTempreture();
  p = LPS.getPressure();
  h = LPS.AltitudeCalc(pressure_origin, p);

  pressure[0] = t;
  pressure[1] = p;
  pressure[2] = h;

  saveLog("hori.csv", pressure, 3);

  Serial.print("t = "); Serial.print(t);
  Serial.print("\tp = "); Serial.print(p);
  Serial.print("\th = "); Serial.println(h);
}
/*
  ///////////超音波センサ//////////////

  float dis_sonic;
  dis_sonic = measure_sonic();
  Serial.print("\t");
  Serial.print(dis_sonic);
  Serial.println(" m");
  delay(500);
  }
*/
/*
  void setup() {
  Serial.begin(9600);
  LPS331AP_init();
  }
  void loop() {
  long P;
  short T;
  float p, t;

  P = LPS331AP_read(LPS331AP_P_H);
  P = (P << 8) | LPS331AP_read(LPS331AP_P_L);
  P = (P << 8) | LPS331AP_read(LPS331AP_P_LL);

  T = LPS331AP_read(LPS331AP_T_H);
  T = (T << 8) | LPS331AP_read(LPS331AP_T_L);

  p = P;
  p = p/4096.0;

  t = T;
  t = 42.5 + t/480.0;

  Serial.print(P);    // pressure (reading)
  Serial.print(" ");
  Serial.print(T);    // temperature (reading)
  Serial.print(" ");
  Serial.print(p);  // pressure in [mbar]/[hPa]
  Serial.print(" ");
  Serial.println(t);  // temprerature in [`C]

  delay(1000);
  }
*/

