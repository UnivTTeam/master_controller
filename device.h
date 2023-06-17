#pragma once

// センサ値
namespace SensorValue {
extern volatile float gyro_theta;
extern volatile float optical_flow_vx;
extern volatile float optical_flow_vy;
}

// 指令値
namespace CommandValue {
extern volatile bool slave_emergency;
extern volatile float wheel_vx;
extern volatile float wheel_vy;
extern volatile float wheel_vw;
}

// デバイスの初期化
void setupDevice();

// IMUの読み込み
void getIMU();
bool imuNotCaliblated();

// デバイス読み込み
void readDevice();

// 指令値の送信
void sendDataToChild();
