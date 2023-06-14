#include "device.h"
#include "flow.h"
#include "params.h"
#include "common.h"

#include <MadgwickAHRS.h>
Madgwick MadgwickFilter;

#include <Arduino.h>

#include <Wire.h>
#include <SparkFunLSM9DS1.h>

LSM9DS1 imu;
#define LSM9DS1_M 0x1E   // コンパスのI2C初期アドレス
#define LSM9DS1_AG 0x6B  // 加速度とジャイロのI2C初期アドレス

#define I2C_DEV_ADDR 0x34

// センサ値
namespace SensorValue {
volatile float imu_yaw = 0.0f;
volatile float optical_flow_vx = 0.0f;
volatile float optical_flow_vy = 0.0f;
}

float imu_9dof[13];   //ax,ay,az,gx,gy,gz,mx,my,mz,temperature,roll,pitch,yaw

float imu_yaw = 0;

void setupIMU() {
  MadgwickFilter.begin(100);  //100Hz、yaw角を求めるフィルターの起動
  //IMU用ここから
  if (imu.begin(LSM9DS1_AG, LSM9DS1_M, Wire) == false)  //IMUの起動を判定
  {
    Serial.println("Failed to communicate with LSM9DS1.");//接続できてなかったら進まない
    while (1) {};
  }
}

void setupDevice() {
  Serial.begin(115200);
  Serial2.begin(19200, SERIAL_8N1, 19, 18);//オプティカルフロー用シリアル通信
  Wire.begin();
  setupIMU();
}

bool imu_calibration_now = true;
float stime = 0.0f;
const float imu_calibration_time_us = Params::imu_calibration_time_sec * 1000.0f * 1000.0f;
const float imu_calibration_start_time_us = Params::imu_calibration_start_time_sec * 1000.0f * 1000.0f;
float imu_drift = 0.0f;

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

  // MadgwickFilterの更新
  static float stime_offset = micros() + imu_calibration_start_time_us;//IMUのドリフト補正用に時間を図ってる
  float stime = micros() - stime_offset;
  MadgwickFilter.updateIMU(
      imu_9dof[3] / 2.048f, 
      imu_9dof[4] / 2.048f, 
      imu_9dof[5] / 2.048f, 
      imu_9dof[0] / 16384.0f,
      imu_9dof[1] / 16384.0f, 
      imu_9dof[2] / 16384.0f);

  // キャリブレーション開始時の処理
  static bool imu_calibration_need_start = true;
  static float stime0 = 0.0f;
  static float yaw0 = 0.0f;
  if(imu_calibration_need_start && stime > 0.0f){
    stime0 = stime;
    yaw0 = MadgwickFilter.getYaw() - 180.0f;
    imu_calibration_need_start = false;
  }

  // キャリブ終了したら反映
  if(imu_calibration_now && stime > imu_calibration_time_us){
    float yaw = MadgwickFilter.getYaw() - 180.0f;
    imu_drift = (yaw - yaw0) / (stime - stime0);
    imu_drift += 0.0000000003f;
    imu_calibration_now = false;
  }

  if(imu_calibration_now){
    imu_yaw = 0.0f;
  } else {
    //IMUのデータをフィルターに入れてる、ドリフトの補正も入っている
    imu_yaw = ((MadgwickFilter.getYaw() - 180.0f) - imu_drift * stime) / 180.0f * M_PI;
    static float imu_yaw_offset = imu_yaw;
    imu_yaw -= imu_yaw_offset;
  }
  // Serial.printf("%f %f\n", micros()/1000000.f, imu_yaw);
}

void readDevice() {
  // デバイス情報を読み込み
  SensorValue::imu_yaw = Params::gyro_scale * imu_yaw;
  updateOpticalFlowVelocity();
}


// 指令値
namespace CommandValue {
volatile float wheel_vx = 0.0f;
volatile float wheel_vy = 0.0f;
volatile float wheel_vw = 0.0f;
}

void sendDataToChild() {
  //グローバル変数wheel_vx,wheel_vy,wheel_vwを読み込み
  uint8_t vx = encode_float(CommandValue::wheel_vx, Params::MAX_PARA_VEL);
  uint8_t vy = encode_float(CommandValue::wheel_vy, Params::MAX_PARA_VEL);
  uint8_t vw = encode_float(CommandValue::wheel_vw, Params::MAX_ROT_VEL);

  uint8_t modeid = 0;

  // Serial.printf("%d %d %d\n", vx, vy, vw);

  Wire.beginTransmission(I2C_DEV_ADDR);
  Wire.write('U');
  Wire.write(vx);
  Wire.write(vy);
  Wire.write(vw);
  Wire.write(modeid);
  uint8_t error = Wire.endTransmission(true);
}
