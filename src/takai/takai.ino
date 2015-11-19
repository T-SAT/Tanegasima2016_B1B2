<<<<<<< HEAD

#include<SPI.h>
#include"infrared.h"

void setup()
{
  Serial.begin(9600);
}
void loop()
{
 float dis_infrared;
 dis_infrared = measure_infrared();
  Serial.print("dis_infrared = ");
  Serial.print(dis_infrared);
  Serial.println("m");
  delay(300);
}
=======
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



<<<<<<< HEAD



=======
>>>>>>> d9fc48eb8f4a52ea6032025ca7dfde15dbe6e155
>>>>>>> cba927b2659eb96786c7c16de76edb474ef26112
