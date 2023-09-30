/*不具合
  ・缶を掴んで持ち上げるとき、完全に持ち上がらず、ターンの角度も小さくなることがある(原因がわからないため、ほぼ運)
  ・電池の充電が足りないと全体的にパワーが下がる(このモデルは充電直後を想定)
  ・機体の動きが止まった際、自動で少し動くようにしているが、それでも停止してしまうことがある。これはPWM_control()関数のif (V1 < 70 && V2 < 70)の70(モーターに出力される電圧)を調整しきれてないのでここを変更することで直る？
*/
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

int limit = 0; //停止時間

int U1 = 0; //PID計算結果1
int U2 = 0; //PID計算結果2

float duration = 0;
float distance = 0;
float speed_of_sound = 331.5 + 0.6 * 25; //25℃の音の速度

int V1 = 0; //M1出力電圧
int V2 = 0; //M2出力電圧

int Mode = 1; //動作の種類
int Mode_2_or_4 = 2;

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
  V1 = 130 - U1 - T1;
  if (V1 < 0) V1 = 0;
  V2 = 130 - U2 - T2;
  if (V2 < 0) V2 = 0;
  analogWrite(IN1,V1);
  analogWrite(IN2,0);
  analogWrite(IN3,V2);
  analogWrite(IN4,0);
  delay(50);
  analogWrite(IN1,0);
  analogWrite(IN3,0);
  if (V1 < 120) {
    analogWrite(IN2, V1);
    analogWrite(IN2, 0);
  }
  if (V2 < 120) {
    analogWrite(IN4, V2);
    analogWrite(IN4, 0);
  }
  if (V1 < 70 && V2 < 70){
    limit++;
    if (limit > 20) {  //limitが20になったら自動でちょっと進む
      for (int i = 0; i < 2; i++) {
        analogWrite(IN1, 150);
        analogWrite(IN2,0);
        analogWrite(IN3, 150);
        analogWrite(IN4,0);
        delay(50);
        analogWrite(IN1,0);
        analogWrite(IN3,0);
        delay(50); 
      }
      limit = 0;
  }
  delay(60);  //Ultrasonic_Sensor()を使うときはいらない
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
  if (distance < 7.00 && Mode_2_or_4 == 2) {  //距離7cm未満でMode2に移行
    Mode = Mode_2_or_4;
    Mode_2_or_4 = 4;
  }
  if (distance < 3.00 && Mode_2_or_4 == 4) Mode = Mode_2_or_4;  //距離3cm未満でMode4に移行
}

void Setting_Power_of_Moter() {  //これを前もって呼び出さないとアームが動かない
  analogWrite(IN8, 255);
}

void Raise_Arm() {  //アームを上げる
  digitalWrite(IN9, HIGH);
  digitalWrite(IN10, LOW);
  delay(1500);
}

void Lower_Arm() {  //アームを下ろす
  for (int i = 0; i < 40; i++) {  //ここの繰り返す回数は正直多すぎても大丈夫、時間がかかるだけで置くのが丁寧になる
    digitalWrite(IN9, LOW);
    digitalWrite(IN10, HIGH);
    delay(20);
    digitalWrite(IN10, LOW);
    delay(100);
  }
}

void Close_Servo() {　　//アームのサーボモーターを閉じる
  servo.write(45);
  delay(50); 
}

void Open_Servo() {  //アームのサーボモーターを開く
  servo.write(90);
  delay(50);
}

void Turn() {  //旋回
  for (int i = 0; i < 9; i++) { //繰り返しの回数を調整して角度を変える
    analogWrite(IN1,255);
    analogWrite(IN2,0);
    analogWrite(IN3,0);
    analogWrite(IN4,255);
    delay(50);
    analogWrite(IN1,0);
    analogWrite(IN4,0);
    delay(50);
  }
}

void Show() { //フォトリフレクタの値を表示(テスト用)
  Serial.print("Mode: ");
  Serial.print(Mode);
  Serial.print(", V1: ");
  Serial.print(V1);
  Serial.print(", V2: ");
  Serial.print(V2);
  Serial.print(", U1: ");
  Serial.print(U1);
  Serial.print(", U2: ");
  Serial.print(U2);
  Serial.print(", val1: ");
  Serial.print(val1);
  Serial.print(", val2: ");
  Serial.println(val2);
}

void loop() {
  Show();
  switch (mode) {
    case 1:  //行きのライントレース
      Read_PhotoReflector();
      PID_control1();
      PID_control2();
      PWM_control();
      Ultrasonic_Sensor();
      break;
    case 2:  //缶を掴む
      delay(1000);
      Setting_Power_of_Moter()
      Close_Servo();
      Raise_Arm();
      Mode = 3;
      break;
    case 3:  //180℃旋回
      delay(1000);
      Turu();
      break;
    case 4:  //缶を離す
      delay(1000);
      Setting_Power_of_Moter();
      LowerArm();
      Open_Servo();
      Mode = 5;
      break;
    case 5:
      delay(1000);
  }
}