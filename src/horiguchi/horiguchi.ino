#include <SPI.h>
#include <SD.h>
#include "pressure.h"
#include "sonic.h"
#include "skLPSxxSPI.h"
#include "sd.h"

skLPSxxx LPS(LPS25H, 2);

float pressure_origin;

void setup(){
  Serial.begin(9600);
  pinMode(10,OUTPUT);
  digitalWrite(10,HIGH);
  SD.begin(SD_CSPIN);
  
  LPS.PressureInit() ;
  LPS.PressureRead();
  pressure_origin = LPS.getPressure();
  
}

void loop(){
  float h;
  float t;
  float p;
  float pi;

  pi = 3.14;
  
  saveLog("test.csv", &pi, 1);
   
/*
  LPS.PressureRead();
  t = LPS.getTempreture();
  p = LPS.getPressure();
  h = LPS.AltitudeCalc(pressure_origin, p);
  
  Serial.print("t = "); Serial.print(t);
  Serial.print("\tp = "); Serial.print(p);
  Serial.print("\th = "); Serial.println(h);
*/
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
/*
void setup(){
 Serial.begin(9600);
 sonic_init();
 }
 void loop(){
 float dis_sonic;
 dis_sonic = measure_sonic();
 Serial.print(dis_sonic);
 Serial.println(" m");
 delay(500);
 }
 */


