#include <ros.h>

#include <geometry_msgs/Twist.h>
#include <ros/time.h>
#include <math.h>

#include <TimerFive.h>

#include <MD_MAX72xx.h>
#include <SPI.h>

#define LED  13

// Pin11と12はタイマー割込みで使う

#define PWMA 10   // L側車輪
#define AI2   9
#define AI1   8
#define ACHA  2   // L側エンコーダ割込みピン
#define ACHB 14
  
#define STBY  7

#define PWMB  4   // R側車輪
#define BI2   5
#define BI1   6
#define BCHA 15
#define BCHB  3   // R側エンコーダ割込みピン

#define INTR_TIME_S 0.05  // タイマー割込み間隔

#define WHEEL_RAD 0.025   // ホイール半径
#define WHEEL_SEP 0.186   // トレッド

#define PPS 840           // 7(パルス/回転) * 2(Up/Down) * 60(ギア比) = 840

/*
 * Matrix LED
 */
#define HARDWARE_TYPE MD_MAX72XX::FC16_HW
#define MATRIX_DEVICES 2

#define CLK_PIN  52 // SCK
#define DATA_PIN 51 // MOSI
#define CS_PIN   53 // SS

#define MATRIX_ROWS 8

/*
 *  A：左車輪
 */
volatile int targetA = 0; // 目標のパルス数/割込み
volatile int last_countA = 0;      // 一回前の割込みまでの累積パルス数
volatile int countA = 0;  // 累積パルス数
volatile float itermA = 0;

/*
 * B：右車輪
 */
volatile int targetB = 0; // 目標のパルス数/割込み
volatile int last_countB = 0;      // 一回前の割込みまでの累積パルス数
volatile int countB = 0;  // 累積パルス数
volatile float itermB = 0;

float pulsePerIntr = PPS * INTR_TIME_S; // 最大パルス数/割込み

ros::NodeHandle nh;

float lastTheta = 0.0;
float lastX = 0.0;
float lastY = 0.0;

int countLED = 0;

/*
 * Matrix LED
 */
MD_MAX72XX MAT_LED = MD_MAX72XX(HARDWARE_TYPE, CS_PIN, MATRIX_DEVICES);

uint8_t eyeBallOn[8] = { 0x3c, 0x7e, 0xff, 0xff, 0xff, 0xff, 0x7e, 0x3c };
uint8_t eyeBallOff[8] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };

uint8_t eyeBallLeft[8]   = { 0x3c, 0x7e, 0xff, 0x9f, 0x9f, 0xff, 0x7e, 0x3c };
uint8_t eyeBallCenter[8] = { 0x3c, 0x7e, 0xff, 0xe7, 0xe7, 0xff, 0x7e, 0x3c };
uint8_t eyeBallRight[8]  = { 0x3c, 0x7e, 0xff, 0xf9, 0xf9, 0xff, 0x7e, 0x3c };



void messageCb(const geometry_msgs::Twist& twist) {
  float cmdV = twist.linear.x;    // 目標速度[m/s]
  float cmdW = twist.angular.z;   // 目標角速度[rad/s]
  float wR, wL;                   // モータの回転角速度[rad/s]

  wR = cmdV / WHEEL_RAD + WHEEL_SEP * cmdW / 2.0 / WHEEL_RAD;
  wL = cmdV / WHEEL_RAD - WHEEL_SEP * cmdW / 2.0 / WHEEL_RAD;

  targetA = (int)(wL * pulsePerIntr / M_PI / 2);   // Left
  targetB = (int)(wR * pulsePerIntr / M_PI / 2);   // Right
/*
  char buf[100];
  char tmp_wL[7];
  char tmp_wR[7];
  dtostrf(wL, 5, 2, tmp_wL);
  dtostrf(wR, 5, 2, tmp_wR);
  sprintf(buf, "wL(L): %s  wR(R): %s  targetA: %d  targetB: %d",
    tmp_wL, tmp_wR, targetA, targetB);
  nh.loginfo(buf);
  */
}

ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel", &messageCb);

void setMotor(float value, int pin1, int pin2, int pinPwm) {
  if (value > 0) {
    digitalWrite(pin1, HIGH);
    digitalWrite(pin2, LOW);
    analogWrite(pinPwm, min(max((int)value, 0), 255));
  }
  else if (value < 0) {
    digitalWrite(pin1, LOW);
    digitalWrite(pin2, HIGH);
    analogWrite(pinPwm, min(max((int)((-1) * value), 0), 255));
  }
  else {
    digitalWrite(pin1,  HIGH);
    digitalWrite(pin2,  HIGH);
    digitalWrite(pinPwm, 0);
  }
}

void readEncoderA() {
  bool sa = digitalRead(ACHA);
  bool sb = digitalRead(ACHB);
  if (sa == sb) {
    countA++;
  }
  else {
    countA--;
  }
}

void readEncoderB() {
  bool sa = digitalRead(BCHA);
  bool sb = digitalRead(BCHB);
  if (sa == sb) {
    countB++;
  }
  else {
    countB--;
  }
}

volatile int led13 = LOW;
volatile int eyePattern = 0;

void timerIsr(void)
{
  if ((countLED % 10) == 0) {
    led13 = !led13;
    digitalWrite(LED, led13);
    countLED = 0;

    switch (eyePattern) {
      case 0:
        MAT_LED.control(MD_MAX72XX::UPDATE, MD_MAX72XX::OFF);
        for (uint8_t i = 0; i < MATRIX_ROWS; i++) {
          MAT_LED.setRow(0, 1, i, eyeBallLeft[i]);
        }
        MAT_LED.control(MD_MAX72XX::UPDATE, MD_MAX72XX::ON);
        eyePattern++;
        break;
      case 1:
        MAT_LED.control(MD_MAX72XX::UPDATE, MD_MAX72XX::OFF);
        for (uint8_t i = 0; i < MATRIX_ROWS; i++) {
          MAT_LED.setRow(0, 1, i, eyeBallCenter[i]);
        }
        MAT_LED.control(MD_MAX72XX::UPDATE, MD_MAX72XX::ON);
        eyePattern++;
        break;
      case 2:
        MAT_LED.control(MD_MAX72XX::UPDATE, MD_MAX72XX::OFF);
        for (uint8_t i = 0; i < MATRIX_ROWS; i++) {
          MAT_LED.setRow(0, 1, i, eyeBallRight[i]);
        }
        MAT_LED.control(MD_MAX72XX::UPDATE, MD_MAX72XX::ON);
        eyePattern++;
        break;
      case 3:
        MAT_LED.control(MD_MAX72XX::UPDATE, MD_MAX72XX::OFF);
        for (uint8_t i = 0; i < MATRIX_ROWS; i++) {
          MAT_LED.setRow(0, 1, i, eyeBallCenter[i]);
        }
        MAT_LED.control(MD_MAX72XX::UPDATE, MD_MAX72XX::ON);
        eyePattern = 0;
        break;
    }
  }
  countLED++;
  
  int speed_countA = countA - last_countA;
  int speed_countB = countB - last_countB;
  last_countA = countA;
  last_countB = countB;
  
  int diffA = (speed_countA - targetA);
  int diffB = (speed_countB - targetB);

  itermA += diffA * INTR_TIME_S;
  itermB += diffB * INTR_TIME_S;

  float Kf = 0.3;   // フィードフォワード
  float Kp = 0.4;   // 比例ゲイン
  float Ki = 0.2;   // 積分ゲイン
  float valueA = Kf * targetA - Kp * diffA - Ki * itermA;
  float valueB = Kf * targetB - Kp * diffB - Ki * itermB;

  valueA /= INTR_TIME_S;
  valueB /= INTR_TIME_S;

  setMotor(valueA, AI1, AI2, PWMA);
  setMotor(valueB, BI1, BI2, PWMB);

  char buf[100];
  char valA[10];
  char valB[10];
  dtostrf(valueA, 6, 3, valA);
  dtostrf(valueB, 6, 3, valB);
  sprintf(buf, "cnt(L, R): (%d,%d)  val(L,R): (%s, %s)  tgt(L,R): (%3d,%3d)",
    countA, countB, valA, valB, targetA, targetB);
  nh.loginfo(buf);

  if (targetA == 0 && speed_countA == 0) itermA *= 0.5;
  if (targetB == 0 && speed_countB == 0) itermB *= 0.5;
/*

  float wA;   // 回転角速度
  float wB;   // 回転角速度

  wA = 2 * PI * speed_countA / PPS / INTR_TIME_S;
  wB = 2 * PI * speed_countB / PPS / INTR_TIME_S;

  float v;
  float w;
  v = WHEEL_RAD * (wB + wA) / 2;
  w = WHEEL_RAD * (wB - wA) / WHEEL_SEP;

  lastTheta += w * INTR_TIME_S;
  lastX += v * cos(lastTheta) * INTR_TIME_S;
  lastY += v * sin(lastTheta) * INTR_TIME_S;
  */
}

void setup() {
  Timer5.initialize(50000);    // 50msごとに割込み(20Hz)
  Timer5.attachInterrupt(timerIsr);
  
  nh.getHardware()->setBaud(57600);
  nh.initNode();
  nh.subscribe(sub);

  pinMode(ACHA, INPUT);
  pinMode(ACHB, INPUT);
  
  pinMode(BCHA, INPUT);
  pinMode(BCHB, INPUT);

  pinMode(STBY, OUTPUT);
  digitalWrite(STBY, LOW);

  pinMode(PWMA, OUTPUT);
  pinMode(AI1,  OUTPUT);
  pinMode(AI2,  OUTPUT);
  
  pinMode(PWMB, OUTPUT);
  pinMode(BI1,  OUTPUT);
  pinMode(BI2,  OUTPUT);
  
  attachInterrupt(digitalPinToInterrupt(ACHA), readEncoderA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(BCHB), readEncoderB, CHANGE);

  pinMode(LED, OUTPUT);


  MAT_LED.begin();
  MAT_LED.control(MD_MAX72XX::UPDATE, MD_MAX72XX::OFF);
  for (uint8_t i = 0; i < MATRIX_ROWS; i++) {
    MAT_LED.setRow(0, 1, i, eyeBallCenter[i]);
  }
  MAT_LED.control(MD_MAX72XX::UPDATE, MD_MAX72XX::ON);
}

void loop() {
  if (nh.connected()) {
    digitalWrite(STBY, HIGH);
  }
  else {
    digitalWrite(STBY, LOW);
  }
  nh.spinOnce();

  delay(10);
}
