#include <SPI.h>
#include <SD.h>
#include <Servo.h>
#include "pressure.h"
#include "sonic.h"
#include "skLPSxxSPI.h"
#include "sd.h"
#include "servo.h"
#include "gyro.h"
#include "accel.h"

//#define MODE_AGL    //加速度、ジャイロ、気圧
//#define MODE_ACCEL  //加速度
//#define MODE_GYRO   //ジャイロ
#define MODE_LPS    //気圧
//#define MODE_SONIC  //超音波
//#define MODE_SERVO  //サーボ

#define accel_cs 7
#define gyro_cs  9
#define LPS_cs   10


skLPSxxx LPS(LPS331AP, LPS_cs ); //圧力センサの型番の設定
float pressure_origin;

#ifdef MODE_AGL
void setup() {
  Serial.begin(9600);
  pinMode(10, OUTPUT);
  SPI.begin();

  init_accel(accel_cs);
  init_gyro(gyro_cs);

  LPS.PressureInit() ;
  LPS.PressureRead();
  int j;
  static long f = millis();
  static float setRC_LPS[2] = {0};

  setRC_LPS[0] = LPS.getPressure();

  while ((millis() - f) < 10000) {
    pressure_origin = LPS.getPressure();
    setRC_LPS[1] = 0.99 * setRC_LPS[0] + 0.01 * pressure_origin;
    pressure_origin = setRC_LPS[1];
    setRC_LPS[0] = setRC_LPS[1];
  }
}
void loop() {
  float x, y, z;

  measure_accel(&x, &y, &z);
  Serial.print(x);    // X accel (m/sec^2)
  Serial.print("\t");
  Serial.print(y);    // Y accel (m/sec^2)
  Serial.print("\t");
  Serial.print(z);  // Z accel (m/sec^2)
  Serial.print("\t");

  measure_gyro(&x, &y, &z);
  Serial.print(x);    // X axis (deg/sec)
  Serial.print("\t");
  Serial.print(y);    // Y axis (deg/sec)
  Serial.print("\t");
  Serial.print(z);  // Z axis (deg/sec)
  Serial.print("\t");

  delay(10);
  float h;
  float t;
  float p;

  LPS.PressureRead();
  t = LPS.getTempreture();
  p = LPS.getPressure();
  h = LPS.AltitudeCalc(pressure_origin, p);

  Serial.print("t = ,"); Serial.print(t);
  Serial.print(",p =,"); Serial.print(p);
  Serial.print(",h =,"); Serial.print(h);

  static float RC_h[2] = {0};
  RC_h[1] = 0.9 * RC_h[0] + 0.1 * h;

  Serial.print(",RC_h=,"); Serial.println(RC_h[1]);

  RC_h[0] = RC_h[1];

  delay(40);
}
#endif

////////////加速度センサ//////////////////
#ifdef MODE_ACCEL
void setup() {
  Serial.begin(9600);
  pinMode(10, OUTPUT);
  SPI.begin();

  init_accel(accel_cs);
}
void loop() {
  float x, y, z;

  measure_accel(&x, &y, &z);
  Serial.print(x);    // X axis (deg/sec)
  Serial.print("\t");
  Serial.print(y);    // Y axis (deg/sec)
  Serial.print("\t");
  Serial.println(z);  // Z axis (deg/sec)

  delay(10);
}
#endif
////////////////ジャイロセンサ//////////////////
#ifdef MODE_GYRO
void setup() {
  Serial.begin(9600);
  pinMode(10, OUTPUT);
  SPI.begin();

  init_gyro(gyro_cs);
}
void loop() {
  float x, y, z;
  //short x_raw, y_raw, z_raw;

  measure_gyro(&x, &y, &z);
  Serial.print(x);    // X axis (deg/sec)
  Serial.print("\t");
  Serial.print(y);    // Y axis (deg/sec)
  Serial.print("\t");
  Serial.println(z);  // Z axis (deg/sec)

  delay(10);
}
#endif
//////////////圧力センサ/////////////////////
#ifdef MODE_LPS
void setup() {
  Serial.begin(9600);
  pinMode(10, OUTPUT);
  SPI.begin();

  LPS.PressureInit() ;
  LPS.PressureRead();
  int j;
  static long f = millis();
  static float setRC_LPS[2] = {0};

  //setRC_LPS[0] = LPS.getPressure();

  while ((millis() - f) < 10000) {
    pressure_origin = LPS.getPressure();
    setRC_LPS[1] = 0.99 * setRC_LPS[0] + 0.01 * pressure_origin;
    pressure_origin = setRC_LPS[1];
    setRC_LPS[0] = setRC_LPS[1];
  }
}
void loop() {
  float h;
  float t;
  float p;

  LPS.PressureRead();
  t = LPS.getTempreture();
  p = LPS.getPressure();
  h = LPS.AltitudeCalc(pressure_origin, p);

  Serial.print("t = ,"); Serial.print(t);
  Serial.print(",\tp =,"); Serial.print(p);
  Serial.print(",\th =,"); Serial.print(h);

  static float RC_h[2] = {0};
  RC_h[1] = 0.9 * RC_h[0] + 0.1 * h;

  Serial.print(",\tRC_h=,"); Serial.println(RC_h[1]);

  RC_h[0] = RC_h[1];

  delay(40);
}
#endif
/////////////////超音波センサ//////////////////
#ifdef MODE_SONIC
void setup() {
  Serial.begin(9600);
  sonic_init();
}
void loop() {
  float dis_sonic;
  dis_sonic = measure_sonic();

  Serial.print("\t");
  Serial.print(dis_sonic);
  Serial.println(" [m]");
}
#endif
////////////////サーボ//////////////////////
#ifdef MODE_SERVO
void setup() {
  Serial.begin(9600);
}
void loop() {
  move_servo();
}
#endif
//////SDサンプル///////////
/*
  void setup(){
  SD.begin(SD_CSPIN);
  }
  void loop(){
  float x,y,z
  sd[3];
  sd[0]=x ;
  sd[1]=y ;
  sd[2]=z ;

   saveLog(sd.csv,sd,3);
  }

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

