#include <Arduino.h>
#include <PS4Controller.h>
#include <functional>

#include "params.h"
#include "task.h"
#include "operate.h"
#include "device.h"
#include "map.h"

#define PS4_MAC "08:B6:1F:3B:81:AE"  //PS4コントローラのMACアドレス

namespace Task {
enum class Mode : int {
  Emergency = -1,
  Manual = 0,
  Auto = 1,
};

const int switch_minimum_interval = 1000;
struct SwitchUpperTrigger {
  SwitchUpperTrigger() : last_state(false), last_time(millis())
  {
  }

  bool operator()(bool state){
    bool trigger = state ^ last_state;
    last_state = state;

    bool ret = (trigger & state);
    if(ret){
      int t = millis();
      if(t - last_time > switch_minimum_interval){
        last_time = t;
        return true;
      }
    }
    return false;
  }

private:
  bool last_state;
  int last_time;
};

linear::Vec2<float> v_dest(0.0f, 0.0f);
float theta_dest = Params::init_pos.rot.getAngle();

Mode mode = Mode::Manual;
int elevator_state = 0;
float current_time = 0.0f;

std::function<bool()> auto_mode_callback = []() { return true; };

SwitchUpperTrigger kumade_wrapper = SwitchUpperTrigger();
SwitchUpperTrigger l1_wrapper = SwitchUpperTrigger();
SwitchUpperTrigger r1_wrapper = SwitchUpperTrigger();

void setupTask() {
  PS4.begin(PS4_MAC);
}

float stickToVelocity(int8_t stick_input){
  float ratio = static_cast<float>(stick_input) / 128.0f;
  if(abs(ratio) < 0.1f){
    ratio = 0.0f;
  }
  return 500.0f * ratio;
}

void taskCallback() {
  current_time = micros() / 1000000.0f;
  // 緊急停止処理
  if(PS4.Touchpad()){
    mode = Mode::Emergency;
  }  
  if(mode == Mode::Emergency){
    CommandValue::wheel_vx = 0.0f;
    CommandValue::wheel_vy = 0.0f;
    CommandValue::wheel_vw = 0.0f;
  }
  
  if(mode != Mode::Emergency){
    // 熊手処理
    if(kumade_wrapper(PS4.Circle())){
      if(elevator_state == 0){
        Serial.printf("熊手を上げる処理1\n");
      }else if(elevator_state == 1){
        Serial.printf("熊手を上げる処理2\n");
      }else if(elevator_state == 2){
        Serial.printf("熊手を上げる処理3\n");
      }
      elevator_state++;      
    }

    // 足回り処理
    v_dest = linear::Vec2<float>(0.0f, 0.0f);
    if(mode == Mode::Manual) {
      // スティック入力
      v_dest.x = stickToVelocity(PS4.LStickX());
      v_dest.y = stickToVelocity(PS4.RStickY());

      // L1 R1による角度微調整
      if (l1_wrapper(PS4.L1())) {
        theta_dest += Params::l1r1_rot_angle;
      }
      if (r1_wrapper(PS4.R1())) {
        theta_dest -= Params::l1r1_rot_angle;
      }

      // モード受付
      if (PS4.L2()) { // L2回転
        mode = Mode::Auto;
        float start_time = current_time;
        float theta_dest0 = theta_dest;
        auto_mode_callback = [&, start_time, theta_dest0](){
          float r = (current_time - start_time) / Params::l2r2_rot_time;
          theta_dest = theta_dest0 + Params::l2r2_rot_angle * min(r, 1.0f);
          return (r >= 1.0f);
        };
      } else if (PS4.R2()) { // R2回転
        mode = Mode::Auto;
        float start_time = current_time;
        float theta_dest0 = theta_dest;
        auto_mode_callback = [&, start_time, theta_dest0](){
          float r = (current_time - start_time) / Params::l2r2_rot_time;
          theta_dest = theta_dest0 - Params::l2r2_rot_angle * min(r, 1.0f);
          return (r >= 1.0f);
        };
      } else if (PS4.Cross()) {
        mode = Mode::Auto;
        float start_time = current_time;
        auto_mode_callback = [&, start_time](){
          float t = current_time - start_time;
          v_dest.x = std::sin(10.0f * t);
          return (t >= 1.0f);
        };
      }
    } else if (mode == Mode::Auto){
      if(auto_mode_callback()){
        mode = Mode::Manual;
      }
    }
    //Serial.printf("%f %f %f %f \n", current_time, v_dest.x, v_dest.y, theta_dest);

    setVelocityFromField(v_dest.x, v_dest.y, theta_dest);
  }

  // Serial.printf("%f %f %f %f %f %f %f\n", 
  //   current_time,
  //   robot_pos.static_frame.pos.x, robot_pos.static_frame.pos.y, robot_pos.static_frame.rot.getAngle(),
  //   robot_pos.dynamic_frame[0].pos.x, robot_pos.dynamic_frame[0].pos.y, robot_pos.dynamic_frame[0].rot  
  // );
}
}