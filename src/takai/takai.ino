#include <SPI.h>
#include "gyro.h"

void setup() {
  init_gyro(SS);
}

void loop() {
  float x, y, z;
  short x_raw, y_raw, z_raw;
  
  get_gyro(&x_raw, &y_raw, &z_raw);
  measure_gyro(&x, &y, &z);

  Serial.print(x_raw);    // X axis (reading)
  Serial.print("\t");
  Serial.print(y_raw);    // Y axis (reading)
  Serial.print("\t");
  Serial.print(z_raw);    // Z axis (reading)
  Serial.print("\t");
  Serial.print(x);    // X axis (deg/sec)
  Serial.print("\t");
  Serial.print(y);    // Y axis (deg/sec)
  Serial.print("\t");
  Serial.println(z);  // Z axis (deg/sec)

  delay(10);
}


