#include <Arduino.h>

#include "params.h"
#include "device.h"
#include "map.h"
#include "operate.h"

void setup() {
  setupDevice();
  initMap();
}

float last_control_time = 0;
void loop() {
  // メインループ
  last_control_time = micros() - Params::control_interval_us;

  // 一定周期ごとに制御を行う
  if (micros() - last_control_time >= Params::control_interval_us) {
    // 時刻情報をアップデート
    last_control_time = micros();

    // デバイス情報を読み込み
    readDevice();

    // 自己位置情報を更新
    updateMap();
    
    // 指令値を更新 (引数いい感じにしてください)
    float vx = 0.0f;
    float vy = 0.0f;
    float theta = 0.0f;
    setVelocityFromField(vx, vy, theta);

    // 指令値を送信
    sendDataToChild();
  }
}
