#include <SPI.h>
#include <SD.h>
#include <SoftwareSerial.h>
#include "skLPSxxSPI.h"
#include "sd.h"
#include "gyro.h"
#include "accel.h"
#include "wireless.h"
#include "fall.h"

//#define MODE_FALL
#define MODE_AGL    //加速度、ジャイロ、気圧
//#define MODE_ACCEL    //加速度+SD
//#define MODE_GYRO   //ジャイロ
//#define MODE_LPS    //気圧
//#define MODE_SONIC  //超音波

#define accel_cs A4
#define gyro_cs  A2
#define LPS_cs   A1

#define OFFSET_NUM 1000

skLPSxxx LPS(LPS331AP, LPS_cs ); //圧力センサの型番の設定
float pressure_origin;

void ReceiveStr(char *str) {
  int i;
  char ch;

  for (i = 0; ch != '!';) {
    if (wirelessAvailable()) {
      ch = receiveData();
      str[i] = ch;
      i++;
    }
  }
  str[i - 1] = '\0';
}

float getDt(void)
{
  static long lastTime = 0;

  long nowTime = millis();
  float time = (float)(nowTime - lastTime);
  time = max(time, 20);  //timeは20[us]以上
  time /= 1000;  //[usec] => [sec]
  lastTime = nowTime;

  return ( time );
}

float RC_LPS[2] = {0};

void SPI_init()
{
  digitalWrite(10, HIGH);
  pinMode(10, OUTPUT);
  SPI.begin();
  SPI.setBitOrder(MSBFIRST);
  SPI.setDataMode(SPI_MODE3);
  SPI.setClockDivider(SPI_CLOCK_DIV8); // 16MHz/8 = 2MHz; (max 10MHz)
}

void sensor_init(void)
{
  init_gyro(gyro_cs);
  LPS.PressureInit() ;
  init_accel(accel_cs);
}

#ifdef MODE_FALL
void setup()
{
  int j;
  static long f = millis();
  static float setRC_LPS[2] = {0};
  float p;

  Serial.begin(9600);
  pinMode(PARAPIN, OUTPUT);
  wireless_init();
  sensor_init();
  setRC_LPS[0] = LPS.getPressure();
  while ((millis() - f) < 10000) {
    pressure_origin = LPS.getPressure();
    setRC_LPS[1] = 0.99 * setRC_LPS[0] + 0.01 * pressure_origin;
    pressure_origin = setRC_LPS[1];
    setRC_LPS[0] = setRC_LPS[1];
  }

  char str[100];
  Serial.println("check");
  TransferStr("Ready...");
  while (strcmp(str, "START") != 0) {
    ReceiveStr(str);
  }

  TransferStr("Start!");
  p = LPS.getPressure();
  RC_LPS[0] = 0;
}

void loop()
{
  float data[10];
  float x, y, z;
  int CurrentState;
  static float angle = 0;
  float dt;

  dt = getDt();
  measure_accel(&x, &y, &z);
  data[0] = x;
  data[1] = y;
  data[2] = z;
  Serial.print(x);
  Serial.print("\t");
  Serial.print(y);
  Serial.print("\t");
  Serial.print(z);
  Serial.print("\t");

  measure_gyro(&x, &y, &z);
  angle += z * dt;
  data[3] = x;
  data[4] = y;
  data[5] = z;
  Serial.print(x, 7);
  Serial.print("\t");
  Serial.print(y, 7);
  Serial.print("\t");
  Serial.print(z, 7);
  Serial.print("\t");

  delay(10);
  float h;
  float t;
  float p;

  LPS.PressureRead();
  t = LPS.getTempreture();
  p = LPS.getPressure();
  h = LPS.AltitudeCalc(pressure_origin, p);
  Serial.print(t);
  Serial.print("\t");
  Serial.print(p);
  Serial.print("\t");
  Serial.print(h);
  Serial.print("\t");
  data[6] = t;
  data[7] = p;
  data[8] = h;

  RC_LPS[1] = 0.9 * RC_LPS[0] + 0.1 * h;
  RC_LPS[0] = RC_LPS[1];

  data[9] = RC_LPS[1];
  CurrentState = check_st(RC_LPS[1]);
  data[10] = (float)CurrentState;

  delay(40);
  transferData(data, 11);
}
#endif

#ifdef MODE_AGL
void setup() {
  int j;
  static long f = millis();
  static float setRC_LPS[2] = {0};

  Serial.begin(9600);
  sensor_init();
  LPS.PressureRead();
  setRC_LPS[0] = LPS.getPressure();

  while ((millis() - f) < 10000) {
    pressure_origin = LPS.getPressure();
    setRC_LPS[1] = 0.99 * setRC_LPS[0] + 0.01 * pressure_origin;
    pressure_origin = setRC_LPS[1];
    setRC_LPS[0] = setRC_LPS[1];
  }
  setRC_LPS[0] = 0.0;
}

void loop() {
  float x, y, z;
  static float angle = 0;
  float dt;

  //L3GD20_display();
  dt = getDt();
  measure_accel(&x, &y, &z);
  /**/
  Serial.print(x);    // X accel (m/sec^2)
  Serial.print("\t");
  Serial.print(y);    // Y accel (m/sec^2)
  Serial.print("\t");
  Serial.print(z);  // Z accel (m/sec^2)
  Serial.print("\t");
  /**/

  measure_gyro(&x, &y, &z);
  angle += z * dt;

  Serial.print(x);    // X axis (deg/sec)
  Serial.print("\t");
  Serial.print(y);    // Y axis (deg/sec)
  Serial.print("\t");
  Serial.print(z);  // Z axis (deg/sec)
  Serial.print("\t");
  Serial.print(angle);
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
  pinMode(SS, OUTPUT);
  SPI.begin();

  SensorInit();
  == == == =
    sensor_init();

  init_accel(accel_cs);
  SD.begin(SD_CSPIN);
}
void loop() {
  float x, y, z;

  measure_accel(&x, &y, &z);
  Serial.print(x);    // X axis (deg/sec)
  Serial.print("\t");
  Serial.print(y);    // Y axis (deg/sec)
  Serial.print("\t");
  Serial.println(z);  // Z axis (deg/sec)
  /*
    float sd[3];
    sd[0] = x ;
    sd[1] = y ;
    sd[2] = z ;

    saveLog("21g.csv", sd, 3);
  */
  delay(10);
}
#endif
////////////////ジャイロセンサ//////////////////
#ifdef MODE_GYRO
void setup() {
  delay(1000);
  Serial.begin(9600);
  pinMode(SS, OUTPUT);
  digitalWrite(SS, HIGH);
  SPI.begin();
  pinMode(A1, OUTPUT);
  pinMode(A2, OUTPUT);
  pinMode(A4, OUTPUT);
  digitalWrite(A1, HIGH);
  digitalWrite(A2, HIGH);
  digitalWrite(A4, HIGH);
  sensor_init();

  init_gyro(gyro_cs);
  wireless_init();
}
void loop() {
  float x, y, z;
  //short x, y, z;
  static float angle = 0;
  float dt;
  //short x_raw, y_raw, z_raw;

  dt = getDt();
  measure_gyro(&x, &y, &z);
  angle += z * dt;
  Serial.print(x);    // X axis (deg/sec)
  Serial.print("\t");
  Serial.print(y);    // Y axis (deg/sec)
  Serial.print("\t");
  Serial.print(z);  // Z axis (deg/sec)
  Serial.print("\t");
  Serial.print(angle);
  Serial.print("\t");
  Serial.print(dt);
  Serial.println("\t\t");
  delay(50);
}
#endif

//////////////圧力センサ/////////////////////
#ifdef MODE_LPS
void setup() {
  Serial.begin(9600);
  sensor_init();

  SensorInit();
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
  setRC_LPS[0] = 0;
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

  Serial.print(dis_sonic);
  Serial.println(" [m]");

  delay(500);
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
*/
