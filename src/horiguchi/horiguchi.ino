#include <SPI.h>
#include <SD.h>
#include <Servo.h>
#include "pressure.h"
#include "sonic.h"
#include "skLPSxxSPI.h"
#include "sd.h"
#include "servo.h"
#include "gyro.h"

//////CS pressure 10 accel 7 sd 3 gyro 9///////

skLPSxxx LPS(LPS25H, 3);  //圧力センサの型番の設定

float pressure_origin;
float pressure_offsetsum;
float pressure_offset;

void push(float *buffer, int num, float data)
{
  int i;

  for (i = 0; i < num - 1; i++)
    buffer[i + 1] = buffer[i];

  buffer[0] = data;
}

float sum_buffer(float *buffer, int num)
{
  int i;
  float tmp = 0.0;

  for (i = 0; i < num; i++)
    tmp += buffer[i];

  return (tmp);
}

void setup() {
  Serial.begin(9600);
  pinMode(10, OUTPUT);
  /*
    /////////////SD設定部///////////////
    SD.begin(SD_CSPIN);
    //////////////超音波センサ設定部///////////////////
    sonic_init();
  */
  ////////////圧力センサ設定部////////////////
  LPS.PressureInit() ;
  LPS.PressureRead();
  int j;
  static long f = millis();
  while ((millis() - f) < 10000) {
    pressure_offsetsum = LPS.getPressure();
    static float x[2] = {0};
    x[1] = 0.99 * x[0] + 0.01 * pressure_offsetsum;
    pressure_offsetsum = x[1];
    x[0] = x[1];
  }
  pressure_offset = pressure_offsetsum;
  pressure_origin = pressure_offset;
  //Serial.print("pressure_offset="); Serial.print(pressure_offset);
  //delay(5000);
    ////////////ジャイロセンサ設定部////////////
    //  init_gyro(9);

}

void loop()
{
  /*
    //////////サーボ//////////
    move_servo();
  */

  // float falltest[4];
  //float pressure[3];

  ///////////////圧力センサ//////////////////
  float h;
  float t;
  float p;

  LPS.PressureRead();
  t = LPS.getTempreture();
  p = LPS.getPressure();
  h = LPS.AltitudeCalc(pressure_origin, p);

  //falltest[0] = t;
  //falltest[1] = p;
  //falltest[2] = h;

  //pressure[0] = t;
  //pressure[1] = p;
  //pressure[2] = h;


  // saveLog("a.csv", pressure, 3);

  Serial.print("t = ,"); Serial.print(t);
  Serial.print(",p =,"); Serial.print(p);
  Serial.print(",h =,"); Serial.print(h);
  /*
    const int num = 100;

    static float b[num];
    float sum, ave;

    push(b, num, h);
    sum = sum_buffer(b, num);
    ave = sum / (float)num;

    Serial.print(",ave=,"); Serial.println(ave, 6);
  */
  static float y[2] = {0};
  y[1] = 0.9 * y[0] + 0.1 * h;

  /* フィルタ出力を使った処理 */

  Serial.print(",y=,"); Serial.println(y[1]);

  y[0] = y[1];


  /*
    ///////////超音波センサ//////////////

    float dis_sonic;
    dis_sonic = measure_sonic();

    falltest[3] = dis_sonic;

    saveLog("mura6.csv", falltest, 4);

    Serial.print("\t");
    Serial.print(dis_sonic);
    Serial.println(" m");

    float x, y, z;
    float gyro[3];

    gyro[0] = x;
    gyro[1] = y;
    gyro[2] = z;

    saveLog("hori7.csv", gyro, 3);

    measure_gyro(&x, &y, &z);
    Serial.print(x);    // X axis (deg/sec)
    Serial.print("\t");
    Serial.print(y);    // Y axis (deg/sec)
    Serial.print("\t");
    Serial.println(z);  // Z axis (deg/sec)
  */
  delay(40);

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

