#include <SPI.h>
#include "gyro.h"
#include "accel.h"


void setup() {
  Serial.begin(9600);
  SPI.begin();
  pinMode(SS, OUTPUT);
  digitalWrite(SS, HIGH);

  init_gyro(8);
  init_accel(3);
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
  Serial.print("\t");
  
  measure_accel(&x, &y, &z);
  Serial.print(x);    // X axis (deg/sec)
  Serial.print("\t");
  Serial.print(y);    // Y axis (deg/sec)
  Serial.print("\t");
  Serial.println(z);  // Z axis (deg/sec)


  delay(10);
}






