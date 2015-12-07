/*
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
 */
/*
#include <SPI.h>
#include "gyro.h"
#include "accel.h"


void setup() {
  Serial.begin(9600);
  SPI.begin();
  pinMode(SS,OUTPUT);


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
  Serial.print(z);  // Z axis (deg/sec)
  Serial.print("\t\t");

  measure_accel(&x, &y, &z);
  Serial.print(x);    // X axis (deg/sec)
  Serial.print("\t");
  Serial.print(y);    // Y axis (deg/sec)
  Serial.print("\t");
  Serial.println(z);  // Z axis (deg/sec)


  delay(10);
}
*/

#include <SPI.h>
#include "gyro.h"
#include <TinyGPS++.h> //GPSデータを受信し終わるまで待機GPS++を使用するためにヘッダファイルをインクルード
#include <SoftwareSerial.h> //SoftwareSerialを使用するためにヘッダファイルをインクルード

////////////モータ制御ピン/////////////
#define LFPin 5  //電圧をかけると左のモータが正転するマイコンのピン番号をLFPinとして設定
#define LBPin 4  //電圧をかけると左のモータが後転するマイコンのピン番号をLBPinとして設定
#define RFPin 6  //電圧をかけると右のモータが正転するマイコンのピン番号をRFPinとして設定
#define RBPin 7  //電圧をかけると右のモータが後転するマイコンのピン番号をRBPinとして設定
////////////////////////////////////////

/////////////ソフトウェアシリアルで使うピン/////////////////////////////////////
#define SSRX 10    //ソフトウェアシリアルでrxとして使用するピンをSSRXとして設定
#define SSTX  9    //ソフトウェアシリアルでtxとして使用するピンをSSTXとして設定
///////////////////////////////////////////////////////////////////////////////////


TinyGPSPlus gps;                //座標関係の各種演算をするTinyGPSPlusのオブジェクトを生成
SoftwareSerial ss(SSRX, SSTX);  //ソフトウェアシリアルのオブジェクトssをrxピンをSSRX(ピン10),txピンをSSTX(ピン9)で設定し、生成する


//目標の角速度
float G_gyro=0;
//今の角速度
float current_gyro;
//神の比例値ｐ
const float p_gain = 0.6;//例定数。
const int max_str = 255;  //モータに入力する値の上限値
const int min_str = 0;   //モータに入力する値の下限値
const int Lnstr = 255;     //ニュートラル（曲がらず直進する）で進むときの左右のモータに与える入力値
const int Rnstr = 255;

int motor_control(int motorL, int motorR)
{
  if(motorL == 0 && motorR == 0) {
    digitalWrite(LFPin, LOW);
    digitalWrite(LBPin, LOW);
    digitalWrite(RFPin, LOW);
    digitalWrite(RBPin, LOW);
  }
  else if (motorL >= 0 && motorR <= 0) {
    analogWrite(LFPin, motorL);
    digitalWrite(LBPin, LOW);
    digitalWrite(RFPin, LOW);
    analogWrite(RBPin, -motorR);
  }
  else if (motorL <= 0 && motorR >= 0) {
    digitalWrite(LFPin, LOW);
    analogWrite(LBPin, -motorL);
    analogWrite(RFPin, motorR);
    digitalWrite(RBPin, LOW);

  }
  else if (motorR <= 0 && motorL <= 0) {
    digitalWrite(LFPin, LOW);
    analogWrite(LBPin, -motorL);
    digitalWrite(RFPin, LOW);
    analogWrite(RBPin, -motorR);
  }
  else {
    analogWrite(LFPin, motorL);
    digitalWrite(LBPin, LOW);
    analogWrite(RFPin, motorR);
    digitalWrite(RBPin, LOW);
  }

  return (0);
}

void gelay(unsigned long ms)
{
  unsigned long start = millis();
  do
  {
    while (ss.available())//ポストの中身確認
      gps.encode(ss.read());//中身ありで読む
  } 
  while (millis() - start < ms);//1秒待ってやろう
}

void setup() {
    // put your setup code here, to run once:
  //モータ制御用のピンを出力で使用できるように設定
  pinMode(LFPin, OUTPUT);
  pinMode(LBPin, OUTPUT);
  pinMode(RFPin, OUTPUT);
  pinMode(RBPin, OUTPUT);

  //ハードウェアシリアル（備え付けのシリアル通信窓口)を9600[bps]で開始する
  Serial.begin(9600);
  //ソフトウェアシリアル通信をボーレート9600[bps]で開始する
  ss.begin(9600);

  gelay(1000); //GPSデータを受信し終わるまで待機。だいたい１秒かかる

  //北の方位から見た機体の角度を求めるため、原点座標から10秒間前進して離れる（原点座標にとどまると機体の角度が求められない)
  motor_control(255, 255);
  delay(30000);
  motor_control(0, 0);
  
  SPI.begin();
  pinMode(SS, OUTPUT);
  digitalWrite(SS, HIGH);

  init_gyro(8);
}

void loop() {
    float control_value;
    float x,y,z;
    
    measure_gyro(&x, &y, &z);
    current_gyro = z;
    current_gyro = G_gyro - current_gyro;
    
    Serial.print("current_gyro = "); 
    Serial.println(current_gyro);
    control_value = p_gain * current_gyro;
 
    if (control_value < 0)
    control_value = -constrain(abs(control_value), min_str, max_str);
    else
    control_value = constrain(abs(control_value), min_str, max_str);
    
    Serial.print("control_value = "); 
    Serial.println(control_value);
 
    if (control_value < 0)
    motor_control( Lnstr - abs(control_value ) ,Rnstr);
    else
    motor_control( Lnstr , Rnstr - abs(control_value));

    delay(5000);
    motor_control( Lnstr, Rnstr);
    delay(6000);
}


