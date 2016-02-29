#include <SPI.h>
#include <SD.h>
#include <TinyGPS++.h>
#include <SoftwareSerial.h>
#include "gyro.h"
#include "skLPSxxSPI.h"
#include "accel.h"
#include "motor.h"
#include "fall.h"
#include "wireless.h"
#include "sd.h"

#define GOAL_LAT 35.515819
#define GOAL_LON 134.171783 
#define DEG2RAD (PI/180.0)
#define RAD2DEG (180.0/PI)
#define GOAL_RANGE 1.0

#define accel_cs A3
#define gyro_cs  A1
#define LPS_cs   A2

#define PGAIN 3.0
#define IGAIN 0.0

float DestLat, DestLon, OriginLat, OriginLon;

skLPSxxx LPS(LPS25H, LPS_cs);
TinyGPSPlus gps;
SoftwareSerial ss(9, 8); //rx=9,tx=8

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
  SPI_init();
  init_gyro(gyro_cs);
  LPS.PressureInit() ;
  init_accel(accel_cs);
}

float PIDcontrol(float command, float current)
{
  float controlValue;
  float error;
  static float i_error = 0.0;

  error = command - current;
  i_error += error;

  controlValue = PGAIN*error + IGAIN*i_error;
  Serial.print("error = ");
  Serial.print(error);
  Serial.print("\t");
  Serial.print("I_error = ");
  Serial.print(i_error);
  Serial.print("\t");
  Serial.print("controlValue = ");
  Serial.println(controlValue);

  return(controlValue);
}

float getDt(void)
{
  float time;
  static float lastTime = (float)millis()/1000.0;
  float currentTime = (float)millis()/1000.0;

  time = currentTime-lastTime;
  lastTime=(float)millis()/1000.0;

  return(time);
}

float AngleNormalization(float angle)
{
  if(angle>=180)
    angle =  angle - 360;
  else if(angle<=-180)
    angle = angle + 360;

  return(angle);
}

void gelay(unsigned long ms)
{
  unsigned long start = millis();
  do 
  {
    while (ss.available())
      gps.encode(ss.read());
  } 
  while (millis() - start < ms);
}

void motor_control(int motorL, int motorR)
{
  if(motorL<0){
    digitalWrite(LDIR,HIGH); 
    motorL = -motorL;
  }
  else digitalWrite(LDIR,LOW);
  if(motorR<0){
    digitalWrite(RDIR,LOW); 
    motorR = -motorR;
  }
  else digitalWrite(RDIR,HIGH);
  motorL = constrain(motorL, 0,255);
  motorR = constrain(motorR, 0,255);
  analogWrite(LPWM,motorL); 
  analogWrite(RPWM,motorR);
}

void setup()
{
  float x, y, z;
  float angle1,angle2,angle3;
  float controlValue;
  float error;
  unsigned long distance;
  static float RC_LPS[2] = {0};
  float p;
  float pressure_origin;
  unsigned long f;
  float data[7];
  float GPSdata[2];
  
  
  Serial.begin(9600);
  ss.begin(9600);
  wireless_init();
  sensor_init();
  
  LPS.PressureRead();
  f = millis();
  RC_LPS[0] = LPS.getPressure();
  while ((millis() - f) < 10000) {
    LPS.getPressure();
    pressure_origin = LPS.getPressure();
    RC_LPS[1] = 0.99 * RC_LPS[0] + 0.01 * pressure_origin;
    pressure_origin = RC_LPS[1];
    RC_LPS[0] = RC_LPS[1];
  }
  RC_LPS[0] = 0;
  
  while(1) {
    measure_gyro(&x, &y, &z);
    data[0] = x;
    data[1] = y;
    data[2] = z;
    float h, t, p;
    LPS.PressureRead();
    data[3] = t = LPS.getTempreture();
    data[4] = p = LPS.getPressure();
    data[5] = h = LPS.AltitudeCalc(pressure_origin, p);
    RC_LPS[1] = 0.9 * RC_LPS[0] + 0.1 * h;
    RC_LPS[0] = RC_LPS[1];
    data[6] = RC_LPS[1];
    transferData(data, 7);
    saveLog("PH.csv", data, 7);
    Serial.print("t=");
    Serial.print(t);
    Serial.print("\t");
    Serial.print("p=");
    Serial.print(p);
    Serial.print("\t");
    Serial.print("RC_LPS[1];=");
    Serial.print(RC_LPS[1]);
    Serial.print("\t");
    Serial.print("z=");
    Serial.println(z);
    if(check_st(RC_LPS[1]) == ST_LAND) {
      release_para();
      break;
    }
  }
  
  delay(10000);
  gelay(2000);
  GPSdata[0]=OriginLat = gps.location.lat();
  GPSdata[1]=OriginLon = gps.location.lng(); 
  saveLog("GPS", GPSdata, 2); 
    Serial.print("OriginLat=");
    Serial.print(OriginLat);
    Serial.print("\t");
    Serial.print("OriginLon=");
    Serial.println(OriginLon);

  distance =
    (unsigned long)TinyGPSPlus::distanceBetween(
  OriginLat,
  OriginLon,
  GOAL_LAT, 
  GOAL_LON);

  if(distance<=GOAL_RANGE)
  {
    Serial.println("goal");
    motor_control(0, 0);
    while(1);
  }

  motor_control(50,50);
  delay(10000);
  motor_control(0, 0);  
  gelay(1000);
  GPSdata[0]=DestLat=gps.location.lat();
  GPSdata[1]=DestLon=gps.location.lng();
  saveLog("GPS", GPSdata, 2); 
  angle1=TinyGPSPlus::courseTo( 
  OriginLat, OriginLon, DestLat, DestLon);
  angle2=TinyGPSPlus::courseTo(
  DestLat, DestLon, GOAL_LAT, GOAL_LON);
  angle3 = angle1 - angle2; 
  angle3 = -DEG2RAD*AngleNormalization(angle3);
  OriginLat = DestLat;
  OriginLon = DestLon;
  controlValue = PIDcontrol(0, angle3);
  controlValue = constrain(controlValue, -77, 77);
  motor_control(50 - controlValue, 50 + controlValue);
}

void loop()
{  
  float x, y, z;
  float originLat, originLon;
  float angle1,angle2,angle3;
  static float angle = 0;
  float dt;
  float controlValue;
  float error;
  unsigned long distance;
  float GPSdata[2];

  //measure_gyro(&x, &y, &z);
  dt = getDt(); 
  //angle += z * dt;
  //angle = DEG2RAD*AngleNormalization(angle);
  gelay(2000);
  angle = 0;
  GPSdata[0]=DestLat=gps.location.lat();
  GPSdata[1]=DestLon=gps.location.lng();  
  saveLog("GPS", GPSdata, 2); 
  
  distance =
    (unsigned long)TinyGPSPlus::distanceBetween(
  DestLat,
  DestLon,
  GOAL_LAT, 
  GOAL_LON);

  if(distance<=GOAL_RANGE)
  {
    Serial.println("goal");
    motor_control(0, 0);
    while(1);
  }


  angle1 = TinyGPSPlus::courseTo( 
  OriginLat, OriginLon, DestLat, DestLon); 
  angle2 = TinyGPSPlus::courseTo(
  OriginLat, OriginLon, GOAL_LAT, GOAL_LON);

  angle3 = angle1 - angle2;
  angle3 = -DEG2RAD*AngleNormalization(angle3);
  error = angle3 - angle;

  controlValue = PIDcontrol(0, error);
  controlValue = constrain(controlValue, -77, 77);
  Serial.print("controlValue = ");
  Serial.println(controlValue);
  motor_control(50 -controlValue, 50 + controlValue);
  OriginLat = DestLat;
  OriginLon = DestLon;
}














