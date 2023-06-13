#include "device.h"
#include "flow.h"

#include <MadgwickAHRS.h>
Madgwick MadgwickFilter;

#include <Arduino.h>
//#include <PS4Controller.h>

#include <Wire.h>
#include <SparkFunLSM9DS1.h>

LSM9DS1 imu;
#define LSM9DS1_M 0x1E   // コンパスのI2C初期アドレス
#define LSM9DS1_AG 0x6B  // 加速度とジャイロのI2C初期アドレス

#define PS4_MAC "08:B6:1F:3B:81:AE"  //PS4コントローラのMACアドレス

#define PI 3.141592653589793

#define I2C_DEV_ADDR 0x34

float imu_9dof[13];   //ax,ay,az,gx,gy,gz,mx,my,mz,temperature,roll,pitch,yaw

float imu_yaw = 0;
float imu_yaw_offset = 0;

float yaw = 0;

// センサ値
namespace SensorValue {
volatile float imu_yaw = 0.0f;
volatile float optical_flow_vx = 0.0f;
volatile float optical_flow_vy = 0.0f;
}

// 指令値
namespace CommandValue {
volatile float wheel_vx = 0.0f;
volatile float wheel_vy = 0.0f;
volatile float wheel_vw = 0.0f;
}


float delta_vw_little = 0.2;
float delta_vw_big = 0.8;

float input_vx = 0;
float input_vy = 0;
float input_vw = 0;

bool input_vibe = false;
bool input_stop = false;

bool input_elevator_0 = false;
bool input_elevator_1 = false;
bool input_elevator_2 = false;
bool input_elevator_3 = false;

bool elevator_open_0 = false;
bool elevator_open_1 = false;
bool elevator_open_2 = false;
bool elevator_open_3 = false;

//制御間隔(micro sec)
float dt = 20000;  //20ms
unsigned long tmp_time = 0;
float stime = 0;
float stime_offset = 0;

void setupDevice() {
  Serial.begin(115200);
  Serial2.begin(19200, SERIAL_8N1, 19, 18);//オプティカルフロー用シリアル通信
  Wire.begin();
  //IMU用ここから
  if (imu.begin(LSM9DS1_AG, LSM9DS1_M, Wire) == false)  //IMUの起動を判定
  {
    Serial.println("Failed to communicate with LSM9DS1.");//接続できてなかったら進まない
    while (1) {};
  }

  MadgwickFilter.begin(100);  //100Hz、yaw角を求めるフィルターの起動
  getIMU();//IMUからデータ取得
  MadgwickFilter.updateIMU(imu_9dof[3] / 2.048, imu_9dof[4] / 2.048, imu_9dof[5] / 2.048, imu_9dof[0] / 16384.0, imu_9dof[1] / 16384.0, imu_9dof[2] / 16384.0);
  imu_yaw_offset = (MadgwickFilter.getYaw() - 180);  //このときのyaw角を0にする
  stime_offset = millis();//IMUのドリフト補正用に時間を図ってる
  //IMU用ここまで
}

void InputVelocity() {
  input_vx = 10.0;
  input_vy = 20.0;
  
  input_vw = 0;
  bool l1 = true;
  bool r1 = true;
  bool l2 = true;
  bool r2 = true;
  if (l1) {
    input_vw = input_vw + delta_vw_little;
  }
  if (r1) {
    input_vw = input_vw - delta_vw_little;
  }
  if (l2) {
    input_vw = input_vw + delta_vw_big;
  }
  if (r2) {
    input_vw = input_vw - delta_vw_big;
  }

  input_vibe = 0;
  input_stop = 0;
  
  input_elevator_3 = elevator_open_2;
  input_elevator_2 = elevator_open_1;
  input_elevator_1 = elevator_open_0;
  input_elevator_0 = true;

  float drift_th = 0.15;
  float slow_th = 0.6;
  // ドリフト対策
  if (abs(input_vy) < drift_th) {
    input_vy = 0;
  }

  if (abs(input_vx) < drift_th) {
    input_vx = 0;
  }
}

void sendDataToChild() {
  //グローバル変数wheel_vx,wheel_vy,wheel_vwを読み込み
  uint8_t vx = (uint8_t)(CommandValue::wheel_vx * 127 + 128);
  uint8_t vy = (uint8_t)(CommandValue::wheel_vy * 127 + 128);
  uint8_t vw = (uint8_t)(CommandValue::wheel_vw * 127 + 128);

  uint8_t modeid = 0;

  Serial.printf("%d,%d,%d\n", vx, vy, vw);

  Wire.beginTransmission(I2C_DEV_ADDR);
  Wire.write('U');
  Wire.write(vx);
  Wire.write(vy);
  Wire.write(vw);
  Wire.write(modeid);
  uint8_t error = Wire.endTransmission(true);

  if(input_vibe){
    //TODO: do_vibe
  }
  if(input_stop){
    //TODO: do_stop
  }
  if(input_elevator_3){
    //not yet
  }
  if(input_elevator_2){
    //not_yet
  }
  if(input_elevator_1){
    //not_yet
  }
  if(input_elevator_0){
    //not_yet
  }
  
}

void getIMU() {  //IMUの値を取得する関数
  if (imu.gyroAvailable()) {
    imu.readGyro();
  }
  if (imu.accelAvailable()) {
    imu.readAccel();
  }
  if (imu.magAvailable()) {
    imu.readMag();
  }
  if (imu.tempAvailable()) {
    imu.readTemp();
  }
  imu_9dof[0] = imu.calcAccel(imu.ax);
  imu_9dof[1] = imu.calcAccel(imu.ay);
  imu_9dof[2] = imu.calcAccel(imu.az);
  imu_9dof[3] = imu.calcGyro(imu.gx);
  imu_9dof[4] = imu.calcGyro(imu.gy);
  imu_9dof[5] = imu.calcGyro(imu.gz);
  imu_9dof[6] = imu.calcMag(imu.mx);
  imu_9dof[7] = imu.calcMag(imu.my);
  imu_9dof[8] = imu.calcMag(imu.mz);
  imu_9dof[9] = imu.temperature;
}

void readDevice() {
  getIMU();
  //IMUのデータを処理してるとこ
  stime = millis() - stime_offset;
  MadgwickFilter.updateIMU(imu_9dof[3] / 2.048, imu_9dof[4] / 2.048, imu_9dof[5] / 2.048, imu_9dof[0] / 16384.0, imu_9dof[1] / 16384.0, imu_9dof[2] / 16384.0);
  imu_yaw = ((MadgwickFilter.getYaw() - 180) - stime / 100000 * 26.4) / 180 * PI;  //IMUのデータをフィルターに入れてる、ドリフトの補正も入っている
  //IMUのデータ処理ここまで
  SensorValue::imu_yaw = imu_9dof[12];
  
  updateOpticalFlowVelocity();
}
