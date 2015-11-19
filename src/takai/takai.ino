
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
