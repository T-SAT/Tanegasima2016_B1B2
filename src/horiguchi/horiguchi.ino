#include <SPI.h>
#include "pressure.h"

void setup() {
  digitalWrite(SS, HIGH);
  pinMode(SS, OUTPUT);
  
  SPI.begin();
  SPI.setBitOrder(MSBFIRST);
  SPI.setClockDivider(SPI_CLOCK_DIV8); // 8MHz/8 = 1MHz; (max 10MHz)

  Serial.begin(9600);
  while (!Serial) {}

  Serial.println(LPS331AP_read(LPS331AP_WHOAMI), HEX); // should show BB

  LPS331AP_write(LPS331AP_CTRL1, B10010000);
                             //   |||||||+ SPI Mode selection
                             //   ||||||+- DELTA_EN
                             //   |||||+-- BDU: block data update
                             //   ||||+--- DIFF_EN: interrupt circuit enable
                             //   |+++---- ODR2, ODR1, ODR0 (1Hz)
                             //   +------- PD: 0: power down, 1: active
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
