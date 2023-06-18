#include <Arduino.h>

#include "params.h"
#include "device.h"
#include "map.h"
#include "task.h"

void setup() {
  setupDevice();
  initMap();
  Task::setupTask();
}

constexpr float CONTROL_INTERVAL_US = Params::CONTROL_INTERVAL_MS * 1000.0f;

void loop() {
  // 現在時刻
  static float last_control_time_us = micros() - CONTROL_INTERVAL_US;
  float current_time_us = micros();
  float interval_us = current_time_us - last_control_time_us;

  if (interval_us >= CONTROL_INTERVAL_US) {
    // 時刻情報アップデート
    last_control_time_us = current_time_us;
    Params::control_interval_sec = interval_us / (1000.0f * 1000.0f);
    Params::current_time = current_time_us / (1000.0f * 1000.0f);

    // デバイス情報アップデート
    get_flow();
    getIMU();
    readDevice();

    // 制御
    updateMap();
    Task::taskCallback();

    // 子機に送信
    sendDataToChild();
  }
}
