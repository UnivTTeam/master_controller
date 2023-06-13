#include <Arduino.h>

#include "params.h"
#include "task.h"
#include "operate.h"
#include "map.h"

namespace Task {
float t = 0.0;

void taskCallback() {
  float vx = 0.0f;
  float vy = 0.0f;
  float theta = 0.0f;
  setVelocityFromField(vx, vy, theta);

  Serial.print(t);
  Serial.print(" ");
  
  Serial.print(robot_pos.static_frame.pos.x);
  Serial.print(" ");
  Serial.print(robot_pos.static_frame.pos.y);
  Serial.print(" ");
  Serial.print(robot_pos.static_frame.rot.getAngle());
  Serial.print(" ");
  Serial.print(robot_pos.dynamic_frame[0].pos.x);
  Serial.print(" ");
  Serial.print(robot_pos.dynamic_frame[0].pos.y);
  Serial.print(" ");
  Serial.print(robot_pos.dynamic_frame[0].rot);
  Serial.println(" ");

  t += ::Params::control_interval_sec;
}

}