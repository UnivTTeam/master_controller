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
Mode mode = Mode::MapParam;

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

Vec2<float> v_dest(0.0f, 0.0f);
float theta_dest = Params::init_pos.rot.getAngle();
float omega_dest = 0.0f;

float current_time = 0.0f;

std::function<bool()> auto_mode_callback = []() { return true; };

int elevator_step = 0;
std::function<bool()> elevator_callback = []() { return true; };

void reset_elevator_callback()
{
  elevator_callback = [](){
    for(const auto& pin : Params::ELEVATOR_PIN) {
      digitalWrite(pin, LOW);
    }
    return true;
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
  float end_ratio = 1.0f - 1.0f / (Params::rotKp * duration);
  auto_mode_callback = [&, duration, dTheta, start_time, theta0, end_ratio](){
    float ratio = (current_time - start_time) / duration;
    if(ratio < end_ratio){
        theta_dest = theta0 +  dTheta * min(ratio, 1.0f);
        omega_dest = dTheta / duration;
    }else{
        theta_dest = theta0 +  dTheta;
    }
    return (ratio >= 1.0f);
  };
}

void setAutoPara(float duration, float x, float y){
  mode = Mode::Auto;

  float start_time = current_time;
  Vec2<float> init_pos = robot_pos.static_frame.pos;
  Vec2<float> dr(x, y);
  auto_mode_callback = [&, duration, start_time, init_pos, dr](){
    float ratio = min((current_time - start_time) / duration, 1.0f);
    auto dest_pos = init_pos + ratio * dr;

    float Kp = 5.0f;
    auto fb_vel = -Kp * (robot_pos.static_frame.pos - dest_pos);

    auto ff_vel = Vec2<float>(0.0f, 0.0f);
    if(ratio < 1.0f){
      ff_vel = dr / duration;
    }

    v_dest = ff_vel + fb_vel;

    return isStickInterrupt();
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
      CommandValue::wheel_vx = 0.0f;
      CommandValue::wheel_vy = 0.0f;
      CommandValue::wheel_vw = 0.0f;
      for(const auto& pin : Params::ELEVATOR_PIN) {
        digitalWrite(pin, LOW);
      }
      if(PS4.isConnected() && (!emergency_button)){
        mode = Mode::Manual;
      }
    }
  }
  if(mode == Mode::Manual) {
    // モード受付
    if (PS4.L2()) { // L2回転
      setAutoRot(Params::l2r2_rot_time, Params::l2r2_rot_angle);
    } else if (PS4.R2()) { // R2回転
      setAutoRot(Params::l2r2_rot_time, -Params::l2r2_rot_angle);
    } else if (PS4.Triangle()) {
      setAutoGTGT();
    } else if (PS4.Circle()) {
    } else if (PS4.Square()){
      setAutoPara(4.0f, 1760.0f, 1760.0f);
    }
  }
  // 子機にモードを送信
  CommandValue::slave_emergency = (mode == Mode::Emergency);

  // インジケータ―
  if(imuNotCaliblated()){
    int t = (current_time*5.0f);
    digitalWrite(Params::GREEN_LED, t%2);
  }else if(mode == Mode::Emergency){
    digitalWrite(Params::GREEN_LED, false);
  }else if(mode == Mode::Manual || mode == Mode::Auto){
    digitalWrite(Params::GREEN_LED, true);
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
      int target_pin = elevator_step;
      float start_time = current_time;
      elevator_callback = [&, target_pin, start_time]{
        bool is_end = (current_time > start_time + Params::ELEVATOR_TIME);
        for(int i=0; i<Params::ELEVATOR_PIN.size(); i++){
          digitalWrite(Params::ELEVATOR_PIN[i], (!is_end) & (i==target_pin));
        }
        return is_end;
      };
      elevator_step++;
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
    v_dest = Params::MAX_PARA_VEL * readStick();
    setVelocityFromField(v_dest.x, v_dest.y, theta_dest);
  } else if (mode == Mode::Auto){
    if(auto_mode_callback()){
      mode = Mode::Manual;
    }
    setVelocityFromField(v_dest.x, v_dest.y, theta_dest);
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