#include <Servo.h>
#define Kp -1.0f     //Pゲイン
#define Ki 0.0f     //Iゲイン
#define Kd 0.0f     //Dゲイン
#define target1 40 //目標明るさ
#define target2 40 //目標明るさ
Servo servo;

//MODEは常に0
//IN1 = 0, IN2 = 0 で空転
//IN1 = 0, IN2 = 1 で逆転
//IN1 = 1, IN2 = 0 で正転
//IN1 = 1, IN2 = 1 でブレーキ

const int IN1 = 5; //モータードライバ1
const int IN2 = 6; //モータードライバ1
const int IN3 = 9; //モータードライバ2
const int IN4 = 10; //モータードライバ2
const int IN5 = 0; //フォトリフレクタ1 通常600~650
const int IN6 = 1; //フォトリフレクタ2 通常600~650
const int IN7 = 3; //アームのサーボモーター
const int IN8 = 11; //アームのDCモーター アナログ入力
const int IN9 = 12; //アームのDCモーター デジタル入力
const int IN10 = 13; //アームのDCモーター デジタル入力
const int IN11 = 2; //ウルトラソニックセンサーEcho
const int IN12 = 15; //ウルトラソニックセンサーTrig

int val1 = 0; //フォトリフレクタ1の返す値
int val2 = 0; //フォトリフレクタ2の返す値

float pretime = 0; //微分積分のdtに使う現在時刻との差
float integral = 0; //インテグラル
float last_err1 = 0;
float last_err2 = 0;

int limit = 0;

int U1 = 0;
int U2 = 0;

float duration = 0;
float distance = 0;
float speed_of_sound = 331.5 + 0.6 * 25;

void setup() {
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(IN5, OUTPUT);
  pinMode(IN6, OUTPUT);
  servo.attach(IN7, 500, 2400);
  pinMode(IN8, OUTPUT);
  pinMode(IN9, OUTPUT);
  pinMode(IN10, OUTPUT);
  pinMode(IN11, INPUT);
  pinMode(IN12, OUTPUT);
  Serial.begin(9600);
  delay(1000);
}

void Read_PhotoReflector(){ 
  val1 = analogRead(IN5);
  if (val1 > 630) val1 = 630;
  val2 = analogRead(IN6);
  if (val2 > 630) val2 = 630;
}

void PID_control1(){ //モーター1のPID計算
  float dt = (micros() - pretime) / 1000000;
  pretime = micros();
  float err = target1 - val1;
  float P = Kp * err;
  integral += err * dt;
  float I = Ki * integral;
  float diff = (err - last_err1) / dt;
  float D = Kd * diff;
  U1 = P + I + D;
  last_err1 = err;
}

void PID_control2(){ //モーター2のPID計算
  float dt = (micros() - pretime) / 1000000;
  pretime = micros();
  float err = target2 - val2;
  float P = Kp * err;
  integral += err * dt;
  float I = Ki * integral;
  float diff = (err - last_err2) / dt;
  float D = Kd * diff;
  U2 = P + I + D;
  last_err2 = err;
}

void PWM_control() { //PWM制御
  V1 = 125 - U1 - T1;
  if (V1 < 0) V1 = 0;
  V2 = 125 - U2 - T2;
  if (V2 < 0) V2 = 0;
  analogWrite(IN1,V1);
  analogWrite(IN2,0);
  analogWrite(IN3,V2);
  analogWrite(IN4,0);
  delay(50);
  analogWrite(IN1,0);
  analogWrite(IN3,0);
  if (V1 < 120 && limit < 8000) {
    analogWrite(IN2, V1);
    analogWrite(IN2, 0);
    limit++;
  } else if (V2 < 120 && limit < 8000) {
    analogWrite(IN4, V2);
    analogWrite(IN4, 0);
    limit++;
  } else {
    if (limit < 8500) limit = 0;
  }
  //delay(50);  //Ultrasonic_Sensor()を使うときはいらない
}

void Ultrasonic_Sensor() {
  digitalWrite(IN12, LOW); 
  delayMicroseconds(2); 
  digitalWrite(IN12, HIGH);
  delayMicroseconds(10); 
  digitalWrite(IN12, LOW);
  duration = pulseIn(IN11, HIGH); // 往復にかかった時間が返却される[マイクロ秒]

  if (duration > 0) {
    duration = duration / 2; // 往路にかかった時間
    distance = duration * speed_of_sound * 100 / 1000000;
    Serial.print("Distance:");
    Serial.print(distance);
    Serial.println(" cm");
  }

  delay(200);
}

void Setting_Power_of_Moter() {
  analogWrite(IN8, 255);
}

void Raise_Arm() {
  for (int i = 0; i < 15; i++) {
    digitalWrite(IN9, HIGH);
    digitalWrite(IN10, LOW);
    delay(100);
    digitalWrite(IN9, LOW);
    delay(10);
  }
}

void Lower_Arm() {
  for (int i = 0; i < 10; i++) {
    digitalWrite(IN9, LOW);
    digitalWrite(IN10, HIGH);
    delay(100);
    digitalWrite(IN10, LOW);
    delay(10);
  }
}

void Spin_Servo(){ //アームのサーボモーターを動かす
  servo.write(90);
  delay(1000);
  servo.write(45);
  delay(1000);
}

void Show() { //フォトリフレクタの値を表示(テスト用)
  Serial.print("U1: ");
  Serial.print(U1);
  Serial.print(", U2: ");
  Serial.print(U2);
  Serial.print(", val1: ");
  Serial.print(val1);
  Serial.print(", val2: ");
  Serial.println(val2);
}

void loop() {
  //Show();
  /*switch (mode) {
    case 1:
      Read_PhotoReflector();
      PID_control1();
      PID_control2();
      PWM_control();
      Ultrasonic_Sensor();
      break;
    case 2:
      Setting_Power_of_Moter()
      Close_Servo();
      Raise_Arm();
      break;
    case 3:
      Turu();
      break;
    case 4:
      Setting_Power_of_Moter();
      LowerArm();
      Open_Servo();
      break;
  }*/
  Read_PhotoReflector();
  Show();
  PID_control1();
  PID_control2();
  PWM_control();
  //Setting_Power_of_Moter();
  //Raise_Arm();
  //Lower_Arm();
  Ultrasonic_Sensor();
}