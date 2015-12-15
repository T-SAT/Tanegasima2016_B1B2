#include <SoftwareSerial.h>
#include "wireless.h"

SoftwareSerial ss(SS_RX, SS_TX);
void wireless_init()
{
  ss.begin(9600);
  
}

void transferData(float data[], int num)
{
  int i;
  
  for(i = 0; i < num; i++) {
      ss.print(data[i]);
      ss.print(",");    
  }
  
  ss.print(millis());
  ss.println();
}
