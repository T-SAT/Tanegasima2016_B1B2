#include <SPI.h>
#include "gyro.h"

int L3GD20_CS;
//const int SS = 10;      // 必ず 10 番を出力にすること
//const int MOSI = 11;
//const int MISO = 12;
//const int SCK  = 13;

short offsetxGyro, offsetyGyro, offsetzGyro;

const byte L3GD20_WHOAMI = 0x0f;
const byte L3GD20_CTRL1 = 0x20;
const byte L3GD20_CTRL2 = 0x21;
const byte L3GD20_CTRL3 = 0x22;
const byte L3GD20_CTRL4 = 0x23;
const byte L3GD20_CTRL5 = 0x24;
const byte L3GD20_REFERENCE = 0x25;
const byte L3GD20_TEMP = 0x26;
const byte L3GD20_REG = 0x27;
const byte L3GD20_X_L = 0x28;
const byte L3GD20_X_H = 0x29;
const byte L3GD20_Y_L = 0x2A;
const byte L3GD20_Y_H = 0x2B;
const byte L3GD20_Z_L = 0x2C;
const byte L3GD20_Z_H = 0x2D;
const byte L3GD20_FIFO_CTRL = 0x2E;
const byte L3GD20_SRC = 0x2F;
const byte L3GD20_INT1_CFG = 0x30;
const byte L3GD20_INT1_SRC = 0x31;
const byte L3GD20_INT1_TSH_XH = 0x32;
const byte L3GD20_INT1_TSH_XL = 0x33;
const byte L3GD20_INT1_TSH_YH = 0x34;
const byte L3GD20_INT1_TSH_YL = 0x35;
const byte L3GD20_INT1_TSH_ZH = 0x36;
const byte L3GD20_INT1_TSH_ZL = 0x37;
const byte L3GD20_IN1_DURATION = 0x38;

const byte L3GD20_RW = 0x80;
const byte L3GD20_MS = 0x40;


//////write your code///////
void L3GD20_write(byte reg, byte val)
{
  SPI.setDataMode(SPI_MODE3);
  digitalWrite(L3GD20_CS, LOW);
  delayMicroseconds(10);
  SPI.transfer(reg);
  SPI.transfer(val);
  digitalWrite(L3GD20_CS, HIGH);
}

byte L3GD20_read(byte reg)
{
  byte ret = 0;

  SPI.setDataMode(SPI_MODE3);
  digitalWrite(L3GD20_CS, LOW);
  delayMicroseconds(10);
  SPI.transfer(reg | L3GD20_RW);
  ret = SPI.transfer(0);
  digitalWrite(L3GD20_CS, HIGH);

  return ret;
}

void get_gyro(short *x, short *y, short *z)
{
  short X, Y, Z;

  X = L3GD20_read(L3GD20_X_H);
  *x = X = (X << 8) | L3GD20_read(L3GD20_X_L);
  Y = L3GD20_read(L3GD20_Y_H);
  *y = Y = (Y << 8) | L3GD20_read(L3GD20_Y_L);
  Z = L3GD20_read(L3GD20_Z_H);
  *z = Z = (Z << 8) | L3GD20_read(L3GD20_Z_L);
/**
  Serial.print("XH="); Serial.print(L3GD20_read(L3GD20_X_H), HEX); Serial.print("\t");
  Serial.print("XL="); Serial.print(L3GD20_read(L3GD20_X_L), HEX); Serial.print("\t");
  Serial.print("YH="); Serial.print(L3GD20_read(L3GD20_Y_H), HEX); Serial.print("\t");
  Serial.print("YL="); Serial.print(L3GD20_read(L3GD20_Y_L), HEX); Serial.print("\t");
  Serial.print("ZH="); Serial.print(L3GD20_read(L3GD20_Z_H), HEX); Serial.print("\t");
  Serial.print("ZL="); Serial.print(L3GD20_read(L3GD20_Z_L), HEX); Serial.print("\t");
  Serial.print("X="); Serial.print(X, HEX); Serial.print("\t");
  Serial.print("Y="); Serial.print(Y, HEX); Serial.print("\t");
  Serial.print("Z="); Serial.print(Z, HEX); Serial.println("\t");
/**/
}

void measure_gyro(float *x, float *y, float *z)
{
  short x_raw, y_raw, z_raw;
  get_gyro(&x_raw, &y_raw, &z_raw);
  *x = (float)(x_raw - offsetxGyro) * 0.00875; // +-250dps
  //x *= 0.0175;// +-500dps
  //x *= 07;  // +-2000dps
  *y = (float)(y_raw - offsetyGyro) * 0.00875; // +-250dps
  *z = (float)(z_raw - offsetzGyro) * 0.00875; // +-250dps

}

void init_gyro(int CS1)
{
  pinMode(CS1, OUTPUT);
  digitalWrite(CS1, HIGH);

  L3GD20_CS = CS1;

  int i;
  short tmpx, tmpy, tmpz;
  float avrx, avry, avrz;

  avrx = avry = avrz = 0;

  //SPI.setBitOrder(MSBFIRST);
  //SPI.setClockDivider(SPI_CLOCK_DIV8); // 8MHz/8 = 1MHz; (max 10MHz)

  while (!Serial) {
  }

  Serial.print("L3GD20 = ");
  Serial.println(L3GD20_read(L3GD20_WHOAMI), HEX); // should show D4
  L3GD20_write(L3GD20_CTRL1, B00001111);
  /*L3GD20_write(0x21, B00000000);
    L3GD20_write(0x26, B00010000);
    L3GD20_write(0x30, B00000000);
    L3GD20_write(0x36, B00000000);
    L3GD20_write(0x37, B00000000);
  */
  //   |||||||+ X axis enable
  //   ||||||+- Y axis enable
  //   |||||+-- Z axis enable
  //   ||||+--- PD: 0: power down, 1: active
  //   ||++---- BW1-BW0: cut off 12.5[Hz]
  //   ++------ DR1-DR0: ODR 95[HZ
  for (i = 0; i < OFFSET_NUM; i++)
  {
    get_gyro(&tmpx, &tmpy, &tmpz);
    avrx += tmpx;
    avry += tmpy;
    avrz += tmpz;
  }
  offsetxGyro = avrx / OFFSET_NUM;
  offsetyGyro = avry / OFFSET_NUM;
  offsetzGyro = avrz / OFFSET_NUM;
}

void L3GD20_display(void)
{
  char str[100];
  byte address = 0x0F;

  Serial.print(address);
  Serial.print(": ");
  Serial.print(L3GD20_read(address), HEX);
  Serial.print("  ");
  for (address = 0x20; address <= 0x38; address++) {
    Serial.print(address, HEX);
    Serial.print(": ");
    Serial.print(L3GD20_read(address), BIN);
    Serial.print("  ");
  }
  Serial.println();
}

