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
  MapParam = 2,
};
Mode mode = Mode::Manual;

const int switch_minimum_interval = 100;
struct SwitchUpperTrigger {
  SwitchUpperTrigger() : last_state(true), last_time(millis())
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

float current_time = 0.0f;

int elevator_step = 0;
std::function<bool()> elevator_callback = []() { return true; };

void reset_elevator_callback()
{
  elevator_callback = [](){
    for(const auto& pins : Params::ELEVATOR_PINS) {
      for(const auto& pin : pins) {
        digitalWrite(pin, LOW);
      }
    }
    return true;
  };
}

void setElevatorCallback() {
  int target_pin = elevator_step;
  float start_time = current_time;
  elevator_callback = [&, target_pin, start_time]{
    bool is_end = (current_time > start_time + Params::ELEVATOR_TIME);
    for(int i=0; i<Params::ELEVATOR_PINS.size(); i++){
      for(const auto& pin : Params::ELEVATOR_PINS[i]) {
        digitalWrite(pin, (!is_end) & (i==target_pin));
      }
    }
    return is_end;
  };
}

SwitchUpperTrigger up_wrapper = SwitchUpperTrigger();
SwitchUpperTrigger left_wrapper = SwitchUpperTrigger();
SwitchUpperTrigger right_wrapper = SwitchUpperTrigger();
SwitchUpperTrigger down_wrapper = SwitchUpperTrigger();

SwitchUpperTrigger l1_wrapper = SwitchUpperTrigger();
SwitchUpperTrigger r1_wrapper = SwitchUpperTrigger();

void setupTask() {
  PS4.begin(PS4_MAC);
}

float readStickRaw(int8_t x) {
  float ratio = static_cast<float>(x) / 128.0f;
  if(abs(ratio) < 0.16f){
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
bool interruptAutoMode() {
  return (readStick().norm() != 0.0f);
}

Vec2<float> v_dest(0.0f, 0.0f);
float theta_dest = Params::init_pos.rot.getAngle();
float omega_dest = 0.0f;

std::function<bool()> auto_mode_callback = []() { return true; };

/*
enum class AutoControlStep : int {
  Acc = 0,
  MaxVel = 1,
  Dec = 2,
  StopControl = 3,
};
Mode auto_control_mode = AutoControlStep::Acc;
*/

void setAutoRot(bool reverse){
  mode = Mode::Auto;

  float t0 = current_time;
  float theta0 = theta_dest;

  float V = Params::AUTO_CONTROL_ROT_VEL;
  float A = Params::AUTO_CONTROL_ROT_ACC;
  float X = Params::AUTO_CONTROL_ROT_ANGLE;
  float Xacc = V * V / A;
  float Vmax = V;
  float Tvel = 0.0f;
  if(Xacc > X){
    Vmax = std::sqrt(A * X);
    Xacc = X;
  }else{
    Tvel = (X - Xacc) / Vmax; 
  }
  float Tacc = Vmax / A;
  float Ttotal = Tacc + Tvel + Tacc;
  if(reverse){
    V = -V;
    A = -A;
    X = -X;
    Xacc = -Xacc;
    Vmax = -Vmax;
  }
  auto_mode_callback = [=, current_time, theta_dest, omega_dest](){
    float t = current_time - t0;
    bool is_end = false;
    if(t < Tacc){
      theta_dest = theta0 + 0.5f * A * t * t;
      omega_dest = A * t;
    }else if(t < Tacc + Tvel){
      float dt = t - Tacc;
      theta_dest = theta0 + 0.5f * Xacc + Vmax * dt;
      omega_dest = Vmax;
    }else if(t < Ttotal){
      float t_ = Ttotal - t;
      theta_dest = theta0 + X - 0.5f * A * t_ * t_;
      omega_dest = A * t_;
    }else{
      theta_dest = theta0 + X;
      is_end = true;
    }

    return is_end;
  };
}

bool near_end = false;
float stop_control_end_time = 0.0f;
void setAutoPara(float x, float y){
  mode = Mode::Auto;
  near_end = false;

  float t0 = current_time;
  linear::Vec2<float> dr(x, y);
  linear::Vec2<float> ex = (1.0f / dr.norm()) * dr;
  linear::Vec2<float> ey(-ex.y, ex.x);
  
  linear::Vec2<float> r0 = robot_pos.static_frame.pos;

  float V = Params::AUTO_CONTROL_PARA_VEL;
  float A = Params::AUTO_CONTROL_PARA_ACC;
  float X = dr * ex;
  float Xacc = V * V / A;
  float Vmax = V;
  float Tvel = 0.0f;
  if(Xacc > X){
    Vmax = std::sqrt(A * X);
    Xacc = X;
  }else{
    Tvel = (X - Xacc) / Vmax; 
  }
  float Tacc = Vmax / A;
  float Ttotal = Tacc + Tvel + Tacc;
  auto_mode_callback = [=, current_time, v_dest, robot_pos, near_end, stop_control_end_time](){
    bool is_end = false;
    linear::Vec2<float> r = robot_pos.static_frame.pos - r0;
    float x = r * ex;
    float y = r * ey;

    if(!near_end){
      if(x > X - 0.5f * Xacc){
        near_end = true;
        stop_control_end_time = current_time + Tacc;
      }
    }

    float x_dest = 0.0f;
    float vx_ff = 0.0f;
    if(near_end){
      float t_ = stop_control_end_time - current_time;
      if(t_ > 0.0f){
        x_dest = X - 0.5f * A * t_ * t_;
        vx_ff = A * t_;
      }else{
        x_dest = X;
        is_end = true;
      }
    }else if(x < 0.5f * Xacc){
      float t = max(std::sqrt(2.0f * max(x, 0.0f) / A), 0.1f);
      x_dest = x;
      vx_ff = A * t;
    }else{
      x_dest = x;
      vx_ff = Vmax;
    }

    float vx = -Params::paraKp * (x-x_dest) + vx_ff;
    float vy = -Params::paraKp * y;
    v_dest = (vx*ex) + (vy*ey);

    return interruptAutoMode();
  };
}

void setAutoGTGT(){
  mode = Mode::Auto;

  float start_time = current_time;
  auto_mode_callback = [&, start_time](){
    int t = ((current_time - start_time) * 2.0f);
    if(t%2 == 0){
      v_dest.x = -500.0f;
    }else{
      v_dest.x = 450.0f;
    }
    return false;
  };
}

void taskCallback() {
  current_time = micros() / (1000.0f * 1000.0f);
  v_dest = Vec2<float>(0.0f, 0.0f);
  omega_dest = 0.0f;

  // ボタン読み込み
  bool is_end = elevator_callback();
  bool up = up_wrapper(PS4.Up());
  bool left = up_wrapper(PS4.Left());
  bool right = up_wrapper(PS4.Right());
  bool down = up_wrapper(PS4.Down());
  bool l1 = l1_wrapper(PS4.L1());
  bool r1 = r1_wrapper(PS4.R1());

  // モード読み込み
  if(mode == Mode::Emergency || mode == Mode::Manual || mode == Mode::Auto){
    // 緊急停止処理
    bool emergency_button = PS4.Cross();
    if(emergency_button || (!PS4.isConnected()) || imuNotCaliblated()){
      mode = Mode::Emergency;
    }
    if(mode == Mode::Emergency){
      if(PS4.isConnected() && (!emergency_button)){
        mode = Mode::Manual;
      }
    }
  }
  if(mode == Mode::Manual) {
    // モード受付
    if (PS4.L2()) { // L2回転
      setAutoRot(false);
    } else if (PS4.R2()) { // R2回転
      setAutoRot(true);
    } else if (PS4.Triangle()) {
      setAutoGTGT();
    } else if (PS4.Circle()) {
    } else if (PS4.Square()){
      setAutoPara(1760.0f, 1760.0f);
    }
  }
  // 子機にモードを送信
  CommandValue::slave_emergency = (mode == Mode::Emergency || mode == Mode::MapParam);

  // インジケータ―
  if(imuNotCaliblated()){
    int t = (current_time*5.0f);
    digitalWrite(Params::GREEN_LED, t%2);
  }else if(mode == Mode::Emergency){
    digitalWrite(Params::GREEN_LED, false);
  }else if(mode == Mode::Manual){
    digitalWrite(Params::GREEN_LED, true);
  }else if(mode == Mode::Auto){
    int t = (current_time*10.0f);
    digitalWrite(Params::GREEN_LED, t%2);
  }else{
    digitalWrite(Params::GREEN_LED, false);
  }

  // 熊手処理
  if(mode == Mode::Manual || mode == Mode::Auto){
    if(right){
      reset_elevator_callback();
    }
    if(left){
      reset_elevator_callback();
      elevator_step--;
    }

    if(is_end & up){
      Serial.printf("熊手上昇指令 %d\n", elevator_step);
      setElevatorCallback();
      elevator_step++;
    }
  } else if(mode == Mode::Emergency){
    for(const auto& pins : Params::ELEVATOR_PINS) {
      for(const auto& pin : pins) {
        digitalWrite(pin, LOW);
      }
    }
  }

  // 足回り処理
  if(mode == Mode::Manual) {
    // L1 R1による角度微調整
    if (l1) {
      float dtheta = -Params::l1r1_rot_angle;
      robot_pos.static_frame.rot = Rot2<float>(robot_pos.static_frame.rot.getAngle() + dtheta);
    }
    if (r1) {
      float dtheta = Params::l1r1_rot_angle;
      robot_pos.static_frame.rot = Rot2<float>(robot_pos.static_frame.rot.getAngle() + dtheta);
    }

    // スティック入力
    v_dest = Params::MANUAL_MAX_PARA_VEL * readStick();
    setVelocityFromField(v_dest.x, v_dest.y, theta_dest);
  } else if (mode == Mode::Auto){
    if(auto_mode_callback()){
      mode = Mode::Manual;
    }
    setVelocityFromField(v_dest.x, v_dest.y, theta_dest);
  } else if(mode == Mode::Emergency){
    CommandValue::wheel_vx = 0.0f;
    CommandValue::wheel_vy = 0.0f;
    CommandValue::wheel_vw = 0.0f;
  }

  // ログ
  if(mode == Mode::Emergency || mode == Mode::Manual || mode == Mode::Auto){
    Serial.printf("t: %f dest: %f %f %f ",
      current_time, 
      v_dest.x, v_dest.y, theta_dest);
    Serial.printf("pos: %f %f %f vel: %f %f %f\n", 
      robot_pos.static_frame.pos.x, robot_pos.static_frame.pos.y, robot_pos.static_frame.rot.getAngle(),
      robot_pos.dynamic_frame[0].pos.x, robot_pos.dynamic_frame[0].pos.y, robot_pos.dynamic_frame[0].rot  
    );
  } else {
    Serial.printf("t: %f %f ofu: %f %f theta: %f\n", 
      current_time, Params::control_interval_sec, 
      SensorValue::optical_flow_vx, SensorValue::optical_flow_vy, 
      robot_pos.static_frame.rot.getAngle());
  }
}
} // namespace Task