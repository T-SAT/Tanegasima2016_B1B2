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

/////////////ゴール関係//////////////////////
#define GOAL_LAT 35.517723//ゴール地点の緯度をGOAL_LATとして設定
#define GOAL_LON 134.171463//ゴール地点の経度をGOAL_LONとして設定
#define GOAL_RANGE 1
////////////////ゴール地点の範囲[m]////////////

/////////////ソフトウェアシリアルで使うピン/////////////////////////////////////
#define SSRX 10    //ソフトウェアシリアルでrxとして使用するピンをSSRXとして設定
#define SSTX  9    //ソフトウェアシリアルでtxとして使用するピンをSSTXとして設定
///////////////////////////////////////////////////////////////////////////////////


TinyGPSPlus gps;                //座標関係の各種演算をするTinyGPSPlusのオブジェクトを生成
SoftwareSerial ss(SSRX, SSTX);  //ソフトウェアシリアルのオブジェクトssをrxピンをSSRX(ピン10),txピンをSSTX(ピン9)で設定し、生成する

//////////座標関係///////////////
float originFlat_deg;    //原点座標：緯度[°]
float originFlon_deg;    //原点座標：経度[°]
float currentFlat_deg;   //現在の座標：緯度[°]
float currentFlon_deg;   //現在の座標：経度[°]
float goal_angle;       //北の方位を基準にしたゴール地点の角度[°]
/////////////////////////////////
//////////制御変数//////////////////////////
//目標の角速度
float G_gyro=0;
//今の角速度
float current_gyro;
//微小時間
unsigned long d_time = 0;
unsigned long starttime = millis();
///////////////////////////////////////////
////////////////制御定数////////////////////////////
//神の比例値ｐ
const float p_gain = 0.6;//例定数。
const float kd_gain = 0.3;
const float kq_gain = 0.3;
const float kw_gain = 0.3;
const int max_str = 255;  //モータに入力する値の上限値
const int min_str = 0;   //モータに入力する値の下限値
const int Lnstr = 255;     //ニュートラル（曲がらず直進する）で進むときの左右のモータに与える入力値
const int Rnstr = 255;
////////////////////////////////////////////////////


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
/* GPSデータ受信用関数。少なくともmsミリ秒待つ。（これをしないとモータ動かしたときに変な感じになったらしい）
 * 基本的にrecvGPSは使わずこっちを使うように
 * 引数：unsigned long ms:待つ時間[ms]
 * 返り値：なし
 */
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

  originFlat_deg = gps.location.lat(); //受信した緯度を原点座標の緯度に設定
  originFlon_deg = gps.location.lng(); //受信した経度を原点座標の経度に設定

  goal_angle = TinyGPSPlus::courseTo(originFlat_deg, originFlon_deg, GOAL_LAT, GOAL_LON); //北の方位から見たゴール座標の角度を求める

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
    unsigned long int distance_m;  //機体の現在地からゴールまでの距離
    unsigned long int distance_o;  //原点から現在地までの距離
    long int distancetrace;        //現在地から軌跡までの距離
    float current_angle;           //機体から見たゴールの座標(-180～180)[°]
    float control_value;
    float x,y,z;
    
    gelay(1000); //GPSデータを受信し終わるまで待機
    currentFlat_deg = gps.location.lat();  //受信した座標の緯度を現在地の座標の緯度として設定
    currentFlon_deg = gps.location.lng();  //受信した座標の経度を現在地の座標の経度として設定

    //現在地からゴールまでの距離を求める
    distance_m = (unsigned long)TinyGPSPlus::distanceBetween(currentFlat_deg, currentFlon_deg,
    GOAL_LAT, GOAL_LON);
    Serial.print("dis ="); 
    Serial.println(distance_m);
   //求めた距離が0だったらその地点がゴールなので終了
   if (distance_m <= GOAL_RANGE) {
     Serial.println("goal");
     motor_control(0, 0);
     while(1);
   }

   //北の方位から見たときの現在の機体の角度を求める(0
   current_angle = TinyGPSPlus::courseTo(originFlat_deg, originFlon_deg, currentFlat_deg, currentFlon_deg);

   current_angle = goal_angle - current_angle;  //機体からみたときのゴール地点への角度を計算する

   //ゴールから機体までの角度を範囲(180～-180)の範囲に変換
   if(current_angle > 180) current_angle = current_angle - 360;
   else if(current_angle < -180) current_angle = current_angle + 360;

   distance_o = (unsigned long)TinyGPSPlus::distanceBetween(originFlat_deg,originFlon_deg,currentFlat_deg, currentFlon_deg);
   //原点座標から現在の座標までの距離（上)
   distancetrace =distance_o * sin(current_angle);//現在の座標から軌跡までの距離
   
    //角速度
    measure_gyro(&x, &y, &z);
    current_gyro = z;//反時計回りって(ー)の値でたっけ？
    current_gyro = G_gyro - current_gyro;//これいるか？
    
    //最適角速度計算
    G_gyro = current_gyro + d_time * (-kd_gain * distancetrace -kq_gain * current_angle -kw_gain * current_gyro);
    
    //微小時間計算
    d_time = (millis() - starttime) / 1000 - d_time;
    
    
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

}


