#include <Servo.h>
//#include "LCD12864RSPI.h"
#include <SoftwareSerial.h>
#include <DFPlayer_Mini_Mp3.h>
//#include "LCD12864RSPI.h"
#include "happy.h"
#include "sleep1.h"
#include "sleep2.h"
#include "HX711.h"          //调用24bitAD HX711库
#include "U8glib.h"
U8GLIB_ST7920_128X64_4X u8g(18, 16, 17);
HX711 HX711_CH0(4, 5, 2130); //SCK,DT,GapValue
//SCK引脚用于arduino和HX711模块通讯的时序提供
//DT引脚用于从HX711读取AD的数据
//GapValue用于校准输出的重量值，如果数值偏大就加大该值，如果数据偏小就减小该值
long weight = 0;    //定义一个变量用于存放承重的重量，单位为g
float tempp;//记录硬币总数
bool change;//LCD猫咪状态改变
//舵机
Servo myservo1;
Servo myservo2;
Servo myservo3;//lid
int pos1 = 0;
int pos2 = 0;
int pos3 = 0;

int coin = 0;//是否有硬币放置
int from1 = 650;
int to1 = 1400;
int from2 = 850;
int to2 = 1300 ;
int mid2 = 1200;
int mid1 = to1 + mid2 - to2; //保证舵机同时运行
int from3 = 1200;
int to3 = 1470;
//触控防抖
int sensorState;
int lastSensorState = 0;
long lastDebounceTime = 0;
long debounceDelay = 50;
//手柄运动模式
int mode = 0;
//称重
long lastWeight = 0;
long debounceDelayW = 50;
long lastDebounceTimeW = 0;

void draw(void) {
  // graphic commands to redraw the complete screen should be placed here
  u8g.setFont(u8g_font_unifont);
  u8g.setFontPosTop();
  u8g.drawStr(0, 3, "Sum:");
  u8g.setPrintPos(30, 3);
  u8g.print(tempp);
  if (!change) {
    u8g.drawBitmapP( 0, 0, 16, 64, sleep1);
  }
  else {
    u8g.drawBitmapP( 0, 0, 16, 64, happy);
  }

}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  mp3_set_serial (Serial);
  mp3_set_volume (5);

  myservo1.attach(10);
  myservo2.attach(11);
  myservo3.attach(6);
  pinMode(13, OUTPUT);
  pinMode(2, INPUT);

  //舵机回位
  myservo1.writeMicroseconds(from1); //写角度到舵机
  delay(5); //延时15ms让舵机转到指定位置
  myservo2.writeMicroseconds(from2); //写角度到舵机
  delay(5); //延时15ms让舵机转到指定位置
  myservo3.write(from3); //写角度到舵机
  delay(5); //延时15ms让舵机转到指定位置
  LcdRefresh();
  //重量传感器
  HX711_CH0.begin();          //读取传感器支架毛重
  delay(1000);                //延时3s用于传感器稳定
  HX711_CH0.begin();          //重新读取传感器支架毛重用于后续计算
  weight = HX711_CH0.Get_Weight();    //采样当前传感器重量，该重量已经自动去皮，去皮值根据初始化程序中采样的值计算。
  Serial.print(weight);     //串口输出当前重量
  Serial.println(" g");      //单位为g
  lastWeight = weight;
}

void loop() {
  LcdRefresh();

  //触控传感器检测有没有硬币
  int val = digitalRead(2);
  //防抖
  // 一旦检测到数据发生变化，记录当前时间
  if (val != lastSensorState) {
    lastDebounceTime = millis();
  }
  // 等待50ms，再进行一次判断，是否和当前状态相同
  // 如果和当前状态不相同，改变状态
  if ((millis() - lastDebounceTime) > debounceDelay) {
    if (val != sensorState) {
      sensorState = val;
    }
    lastSensorState = val;
  }
  digitalWrite(13, val);
  delay(20);

  //如果有硬币
  if (val) {
    coin = 1;//确定有硬币
    change = !change;//LCD变成开心猫
  }
  if (coin)
  {
    LcdRefresh();
    //放“喵”的叫声
    mp3_play (1);
    delay (1000);
  // mode = int(random(0, 10));
    if (mode == 0)mode1();
    if (mode == 1)mode3();
    if (mode == 2)mode1();
    if (mode == 3)mode4();
    if (mode == 4)mode3();
    if (mode == 5)mode2();
    if (mode == 6)mode2();
    if (mode == 7)mode3();
    mode++;
    if (mode > 7) mode = 0;

    //有无硬币状态reset
    coin = 0;

    //计算掉落的硬币重量并识别硬币种类
    long  valW;
    long coinW = 0;
    valW = HX711_CH0.Get_Weight();
    if (valW != lastWeight) {
      int lastWeight2 = lastWeight;
      //读数稳定
      while (valW != lastWeight) {
        lastDebounceTimeW = millis();
        lastWeight = valW;
        if ((millis() - lastDebounceTimeW) > debounceDelayW) {
          valW = HX711_CH0.Get_Weight();
        }
      }

      coinW = valW - lastWeight2;
      if (coinW >= 6) {
        tempp++;
      }
      else if (coinW >= 3) {
        tempp += 0.5;
      }
    }
    

  //  Serial.print(coinW);     //串口输出当前重量
 //   Serial.println(" coin");      //单位为g
    LcdRefresh();
    //LCD变回睡觉猫
    change = !change;
  }

}
//LCD刷新函数
void LcdRefresh() {
  u8g.firstPage();
  do {
    draw();
  } while ( u8g.nextPage() );
}

//舵机驱动函数
void Sweep(int srv, int from, int to, int usec)
{
  Servo tempS;
  int pos;
  switch (srv) {
    case 1:
      tempS = myservo1;
      break;
    case 2:
      tempS = myservo2;
      break;
    case 3:
      tempS = myservo3;
      break;
  }
  if (from <= to) {
    for (pos = from; pos < to; pos += 1)
    {
      tempS.writeMicroseconds(pos);
      delayMicroseconds(usec);
    }
  }
  else
  {
    for (pos = from; pos >= to; pos -= 1)
    {
      tempS.writeMicroseconds(pos);
      delayMicroseconds(usec);
    }
  }
}
//正常
void mode1()
{
  //舵机开始
  Sweep(3, from3, to3, 1500);
  Sweep(1, from1, to1, 2000);
  Sweep(2, from2, mid2, 1800);
  for (pos1 = to1, pos2 = mid2; pos1 >= mid1, pos2 < to2 ; pos1--, pos2++) {
    myservo1.writeMicroseconds(pos1); //写角度到舵机
    myservo2.writeMicroseconds(pos2);
    delay(1); //延时15ms让舵机转到指定位置
    // delayMicroseconds(5000);
  }
  Sweep(1, mid1, from1, 1800);
  Sweep(2, to2, from2, 1800);
  Sweep(3, to3, from3, 1800);
  //舵机结束
}
//快速
void mode2() {
  //舵机开始
  Sweep(3, from3, to3, 1500);
  Sweep(1, from1, to1, 1500);
  Sweep(2, from2, mid2, 1500);
  for (pos1 = to1, pos2 = mid2; pos1 >= mid1, pos2 < to2 ; pos1--, pos2++) {
    myservo1.writeMicroseconds(pos1); //写角度到舵机
    myservo2.writeMicroseconds(pos2);
    delay(1); //延时15ms让舵机转到指定位置
    // delayMicroseconds(5000);
  }
  Sweep(1, mid1, from1, 500);
  Sweep(2, to2, from2, 500);
  Sweep(3, to3, from3, 500);
  //舵机结束
}
//慢慢靠近硬币，快速返回
void mode3() {
  //舵机开始
  Sweep(3, from3, to3, 1500);
  Sweep(1, from1, to1, 2000);
  delay(800);
  Sweep(2, from2, mid2, 5000);
  for (pos1 = to1, pos2 = mid2; pos1 >= mid1, pos2 < to2 ; pos1--, pos2++) {
    myservo1.writeMicroseconds(pos1); //写角度到舵机
    myservo2.writeMicroseconds(pos2);
    delay(1); //延时15ms让舵机转到指定位置
    // delayMicroseconds(5000);
  }
  Sweep(1, mid1, from1, 500);
  Sweep(2, to2, from2, 500);
  Sweep(3, to3, from3, 500);
  //舵机结束
}
//慢速
void mode4() {
  //舵机开始
  Sweep(3, from3, to3, 2000);
  Sweep(1, from1, to1, 3000);
  delay(800);
  Sweep(2, from2, mid2, 5000);
  for (pos1 = to1, pos2 = mid2; pos1 >= mid1, pos2 < to2 ; pos1--, pos2++) {
    myservo1.writeMicroseconds(pos1); //写角度到舵机
    myservo2.writeMicroseconds(pos2);
   // delay(15); //延时15ms让舵机转到指定位置
     delayMicroseconds(5000);
  }
  Sweep(1, mid1, from1, 5000);
  Sweep(2, to2, from2, 5000);
  Sweep(3, to3, from3, 5000);
  //舵机结束
}

