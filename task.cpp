#include <Arduino.h>
#include <PS4Controller.h>
#include <functional>

#include "params.h"
#include "task.h"
#include "operate.h"
#include "device.h"
#include "map.h"

#define PS4_MAC "08:B6:1F:3B:81:AE"  //PS4コントローラのMACアドレス

using linear::Vec2, linear::Rot2;

namespace Task {
enum class Mode : int {
  Emergency = -1,
  Manual = 0,
  Auto = 1,
};

const int switch_minimum_interval = 100;
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

Vec2<float> v_dest(0.0f, 0.0f);
float theta_dest = Params::init_pos.rot.getAngle();
float omega_dest = 0.0f;

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

float readStickRaw(int8_t x) {
  float ratio = static_cast<float>(x) / 128.0f;
  if(abs(ratio) < 0.1f){
    ratio = 0.0f;
  }
  return ratio;
}
Vec2<float> readStick() {
  return Vec2<float>(
    readStickRaw(PS4.LStickX()),
    readStickRaw(PS4.RStickY())
  );
}
bool isStickInterrupt() {
  Vec2<float> stick = readStick();
  return (stick.x != 0.0f) | (stick.y != 0.0f);
}

void setAutoRot(float duration, float dTheta){
  mode = Mode::Auto;

  float start_time = current_time;
  float theta0 = theta_dest;
  auto_mode_callback = [&, duration, dTheta, start_time, theta0](){
    float ratio = (current_time - start_time) / duration;
    theta_dest = theta0 +  dTheta * min(ratio, 1.0f);
    omega_dest = dTheta / duration;
    return (ratio >= 1.0f);
  };
}

void setAutoPara(float duration, const Vec2<float>& dr){
  mode = Mode::Auto;

  float start_time = current_time;
  Vec2<float> init_pos = robot_pos.static_frame.pos;
  auto_mode_callback = [&, start_time, duration, dr, init_pos](){
    float ratio = min((current_time - start_time) / duration, 1.0f);
    auto dest_pos = init_pos + ratio * dr;

    float Kp = 5.0f;
    auto fb_vel = -Kp * (robot_pos.static_frame.pos - dest_pos);

    auto ff_vel = Vec2<float>(0.0f, 0.0f);
    if(ratio < 1.0f){
      ff_vel = dr / duration;
    }

    v_dest = ff_vel + fb_vel;

    return isStickInterrupt;
  };
}

void setAutoGTGT(){
  mode = Mode::Auto;

  float start_time = current_time;
  auto_mode_callback = [&, start_time](){
    float t = current_time - start_time;
    int T = t;
    if(T%2 == 0){
      v_dest.x = -500.0f;
    }else{
      v_dest.x = 450.0f;
    }
    return isStickInterrupt();
  };
}

void taskCallback() {
  current_time = micros() / (1000.0f * 1000.0f);
  // 緊急停止処理
  bool emergency_button = PS4.Cross();
  if(emergency_button || (!PS4.isConnected())){
    mode = Mode::Emergency;
  }
  if(mode == Mode::Emergency){
    CommandValue::wheel_vx = 0.0f;
    CommandValue::wheel_vy = 0.0f;
    CommandValue::wheel_vw = 0.0f;
    if(PS4.isConnected() && (!emergency_button)){
      mode = Mode::Manual;
    }
  }
  
  if(mode != Mode::Emergency){
    // 熊手処理
    if(kumade_wrapper(PS4.Up())){
      elevator_state++;
      Serial.printf("熊手上昇指令 %d\n", elevator_state);
    }
    using Params::ELEVATOR_PIN;
    for(int i=0; i<ELEVATOR_PIN.size(); i++){
      bool tf = (i < elevator_state);
      for(const auto& pin : ELEVATOR_PIN[i]){
        digitalWrite(pin, tf);
      }
    }

    // 足回り処理
    v_dest = Vec2<float>(0.0f, 0.0f);
    omega_dest = 0.0f;
    if(mode == Mode::Manual) {
      // スティック入力
      v_dest = Params::MAX_PARA_VEL * readStick();

      // L1 R1による角度微調整
      if (l1_wrapper(PS4.L1())) {
        float dtheta = Params::l1r1_rot_angle;
        robot_pos.static_frame.rot = Rot2<float>(robot_pos.static_frame.rot.getAngle() + dtheta);
      }
      if (r1_wrapper(PS4.R1())) {
        float dtheta = -Params::l1r1_rot_angle;
        robot_pos.static_frame.rot = Rot2<float>(robot_pos.static_frame.rot.getAngle() + dtheta);
      }

      // モード受付
      if (PS4.L2()) { // L2回転
        setAutoRot(Params::l2r2_rot_time, Params::l2r2_rot_angle);
      } else if (PS4.R2()) { // R2回転
        setAutoRot(Params::l2r2_rot_time, -Params::l2r2_rot_angle);
      } else if (PS4.Triangle()) {
        setAutoGTGT();
      } else if (PS4.Circle()) {
      }
    } else if (mode == Mode::Auto){
      if(auto_mode_callback()){
        mode = Mode::Manual;
      }
    }
    Serial.printf("%f %f %f %f ", current_time, v_dest.x, v_dest.y, theta_dest);
    Serial.printf("%f %f %f %f %f %f %f\n", 
      current_time,
      robot_pos.static_frame.pos.x, robot_pos.static_frame.pos.y, robot_pos.static_frame.rot.getAngle(),
      robot_pos.dynamic_frame[0].pos.x, robot_pos.dynamic_frame[0].pos.y, robot_pos.dynamic_frame[0].rot  
    );

    setVelocityFromField(v_dest.x, v_dest.y, theta_dest);
  }

  /*
  Serial.printf("%f %f %f %f %f %f %f\n", 
    current_time,
    robot_pos.static_frame.pos.x, robot_pos.static_frame.pos.y, robot_pos.static_frame.rot.getAngle(),
    robot_pos.dynamic_frame[0].pos.x, robot_pos.dynamic_frame[0].pos.y, robot_pos.dynamic_frame[0].rot  
  );
  */
}
}
