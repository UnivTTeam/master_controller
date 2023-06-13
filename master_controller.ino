ko#include <Arduino.h>

#include "params.h"
#include "device.h"
#include "map.h"
#include "task.h"

float last_control_time = 0;

void setup() {
  setupDevice();
  initMap();
  last_control_time = micros() - Params::control_interval_us;
}

void loop() {
  // メインループ
  get_flow();

  // 一定周期ごとに制御を行う
  if (micros() - last_control_time >= Params::control_interval_us) {
    // 時刻情報をアップデート
    last_control_time = micros();

    // デバイス情報を読み込み
    readDevice();

    // 自己位置情報を更新
    updateMap();
    
    // コントローラーに基づいて指令値を設定する
    Task::taskCallback();

    // 指令値を送信
    sendDataToChild();
  }
}
