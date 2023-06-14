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

void loop() {
  using Params::device_interval_us;
  static float last_device_time = micros() - device_interval_us;
  if(micros() - last_device_time >= device_interval_us) {
    last_device_time = micros();
    get_flow();
    getIMU();
  }
  
  using Params::control_interval_us;
  static float last_control_time = micros() - control_interval_us;
  if (micros() - last_control_time >= control_interval_us) {
    last_control_time = micros();
    readDevice();
    updateMap();
    Task::taskCallback();
    sendDataToChild();
  }
}
