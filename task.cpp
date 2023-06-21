#include <Arduino.h>
#include <PS4Controller.h>
#include <functional>
#include <memory>

#include "elevator.h"
#include "params.h"
#include "task.h"
#include "operate.h"
#include "device.h"
#include "map.h"
#include "switch.h"
#include "route.h"

#define PS4_MAC "08:B6:1F:3B:81:AE"  //PS4コントローラのMACアドレス

using linear::Vec2, linear::Rot2, Params::current_time;

namespace Task {
enum class Mode : int {
  Emergency = -1,
  Manual = 0,
  Auto = 1,
  Rot = 2,
  MapParam = 3,
};
Mode mode = Mode::Manual;

SwitchUpperTrigger circle_wrapper = SwitchUpperTrigger();

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
Vec2<float> readLStick() {
  return Vec2<float>(
    readStickRaw(PS4.LStickX()),
    readStickRaw(PS4.LStickY())
  );
}
Vec2<float> readRStick() {
  return Vec2<float>(
    readStickRaw(PS4.RStickX()),
    readStickRaw(PS4.RStickY())
  );
}

std::function<bool()> auto_mode_callback = [](){ return true; };
void setAutoMode()
{
  mode = Mode::Auto;
}

void setRotMode()
{
  mode = Mode::Rot;
}

bool force_emergency = false;
void callForceEmergency()
{
  force_emergency = true;
}

int task_step = -1;
bool auto_x_mode = true;
int checkTaskStep(int i){
  if(task_step == i-1 || task_step == i){
    return true;
  }
  return false;
}
void autoTask()
{
  using Elevator::setElevator;
  if(checkTaskStep(0) && PS4.Up()){
    if(task_step==-1){
      robot_pos = Transform::MultidiffTransform<float, 1>(Params::init_pos);
    }
    static float y0 = robot_pos.static_frame.pos.y;
    float ydiff = robot_pos.static_frame.pos.y - y0;
    auto_mode_callback = Route::GeneralRoute({{0.0f, 1550.0f - ydiff}}, 0, 0, 0.0f);
    task_step = 0;
    auto_x_mode = false;
  }else if(checkTaskStep(1) && PS4.Left()){
    auto_mode_callback = Route::LinearRoute(-1.0f, 0.0f);
    task_step = 1;
    auto_x_mode = true;
  }else if(checkTaskStep(2) && PS4.Up()){
    float theta = 0.0f;
    static float y0 = robot_pos.static_frame.pos.y;
    float ydiff = robot_pos.static_frame.pos.y - y0;
    float y_kumade = Params::ELEVATOR_UP_Y_DIFF - std::abs(ydiff);
    auto_mode_callback = Route::GeneralRoute({{theta-robot_pos.static_frame.rot.getAngle()}, {0.0f, 4600.0f - ydiff}}, 1, 1, y_kumade);
    task_step = 2;
    auto_x_mode = false;
  }else if(checkTaskStep(3) && PS4.Right()){
    auto_mode_callback = Route::LinearRoute(1.0f, 0.0f);
    task_step = 3;
    auto_x_mode = true;
  }else if(checkTaskStep(4) && PS4.Down()){
    float theta = M_PI;
    static float y0 = robot_pos.static_frame.pos.y;
    float ydiff = robot_pos.static_frame.pos.y - y0;
    float y_kumade = Params::ELEVATOR_UP_Y_DIFF - std::abs(ydiff);
    auto_mode_callback = Route::GeneralRoute({{theta-robot_pos.static_frame.rot.getAngle()}, {0.0f, -2300.0f - ydiff}}, 2, 1, y_kumade);
    task_step = 4;
    auto_x_mode = false;
  }else if(checkTaskStep(5) && PS4.Left()){
    auto_mode_callback = Route::LinearRoute(-1.0f, 0.0f);
    task_step = 5;
    auto_x_mode = true;
  }else if(checkTaskStep(6) && PS4.Down()){
    auto_mode_callback = Route::LinearRoute(0.0f, -1.0f);
    task_step = 6;
    auto_x_mode = false;
  }else if(checkTaskStep(7) && PS4.Right()){
    auto_mode_callback = Route::GTGTRoute();
    task_step = 7;
    auto_x_mode = false;
  }
}

bool autoModeGoButton() {
  return PS4.Up() | PS4.Left() | PS4.Down() | PS4.Right() | PS4.Square();
}

void taskCallback() {
  // ボタン読み込み
  bool l1 = l1_wrapper(PS4.L1());
  bool r1 = r1_wrapper(PS4.R1());
  bool circle = circle_wrapper(PS4.Circle());
  bool emergency_button = PS4.Cross();

  // モード読み込み
  if(mode != Mode::MapParam){
    // 緊急停止処理
    if(emergency_button || (!PS4.isConnected()) || imuNotCaliblated()){
      mode = Mode::Emergency;
    }
    if(mode == Mode::Emergency){
      if(PS4.isConnected() && (!emergency_button)){
        mode = Mode::Manual;
      }
    }
  }
  if(force_emergency){
    mode = Mode::Emergency;
  }
  if(mode == Mode::Manual) {
    // モード受付
    if (l1) {
      float dtheta = Params::l1r1_rot_angle;
      robot_pos.static_frame.rot = Rot2<float>(robot_pos.static_frame.rot.getAngle() - dtheta);
    } else if (r1) {
      float dtheta = -Params::l1r1_rot_angle;
      robot_pos.static_frame.rot = Rot2<float>(robot_pos.static_frame.rot.getAngle() - dtheta);
    } else if (PS4.L2()) { // L2回転
      auto_mode_callback = Route::RotRoute(Params::AUTO_CONTROL_ROT_ANGLE);
    } else if (PS4.R2()) { // R2回転
      auto_mode_callback = Route::RotRoute(-Params::AUTO_CONTROL_ROT_ANGLE);
    }
  }
  if(mode == Mode::Manual){
    if(autoModeGoButton()){
      autoTask();
    }
  }
  if(mode == Mode::Auto && !autoModeGoButton()){
    mode = Mode::Emergency;
  }
  if(emergency_button){
    mode = Mode::Emergency;
  }
  // 子機にモードを送信
  CommandValue::slave_emergency = (mode == Mode::Emergency || mode == Mode::MapParam);
  CommandValue::master_step = task_step;

  // インジケータ―
  if(imuNotCaliblated()){
    int t = (current_time*5.0f);
    digitalWrite(Params::GREEN_LED, t%2);
  }else if(mode == Mode::Emergency){
    digitalWrite(Params::GREEN_LED, false);
  }else if(mode == Mode::Manual){
    digitalWrite(Params::GREEN_LED, true);
  }else if(mode == Mode::Auto || mode == Mode::Rot){
    int t = (current_time*10.0f);
    digitalWrite(Params::GREEN_LED, t%2);
  }else{
    digitalWrite(Params::GREEN_LED, false);
  }

  // 熊手処理
  if(mode == Mode::Manual || mode == Mode::Auto || mode == Mode::Rot){
    using namespace Elevator;
    if(circle){
      if(task_step < 0){
        resetElevator();
      } else if(task_step < 2){
        retryElevator(0);
      } else if(task_step < 4){
        retryElevator(1);
      } else{
        retryElevator(2);
      }
    }
    bool is_end = elevatorCallback();
  } else {
    Elevator::stopElevator();
  }

  // 足回り処理
  linear::Vec2<float> last_v_dest = v_dest;
  v_dest = Vec2<float>(0.0f, 0.0f);
  omega_dest = 0.0f;
  if(mode == Mode::Manual) {
    // スティック入力
    auto l = readLStick();
    auto r = readRStick();
    if(l.norm() > 0.0f){
      if(std::abs(l.x) > std::abs(l.y)){
        l.y = 0;
        r.x = 0;
      }else{
        l.x = 0;
        r.y = 0;
      }
    }
    v_dest = Params::MANUAL_MAX_PARA_VEL * l +  0.5f * Params::MANUAL_MAX_PARA_VEL * r;

    // 加速度制限
    linear::Vec2<float> acc = (v_dest - last_v_dest) / Params::control_interval_sec;
    if(acc.norm() > Params::AUTO_CONTROL_PARA_ACC){
      acc = (Params::AUTO_CONTROL_PARA_ACC / acc.norm()) * acc;
      v_dest = last_v_dest + acc * Params::control_interval_sec;
    }
    setVelocityFromField();
  } else if (mode == Mode::Auto || mode == Mode::Rot){
    auto r_diff = Params::control_interval_sec * Params::AUTO_MODE_MANUAL_POS_CHANGE * readRStick();
    if(auto_x_mode){
      r_diff.x = 0.0f;
    }else{
      r_diff.y = 0.0f;
    }
    Route::addRdiff(r_diff);
    if(auto_mode_callback()){
      mode = Mode::Manual;
    }
    setVelocityFromField();
  } else {
    CommandValue::wheel_vx = 0.0f;
    CommandValue::wheel_vy = 0.0f;
    CommandValue::wheel_vw = 0.0f;
  }

  // ログ
  if(mode != Mode::MapParam){
    Serial.printf("t: %f dest: %f %f %f ",
      current_time, 
      v_dest.x, v_dest.y, theta_dest);
    Serial.printf("pos: %f %f %f vel: %f %f %f\n", 
      robot_pos.static_frame.pos.x, robot_pos.static_frame.pos.y, robot_pos.static_frame.rot.getAngle(),
      robot_pos.dynamic_frame[0].pos.x, robot_pos.dynamic_frame[0].pos.y, robot_pos.dynamic_frame[0].rot  
    );
  } else { // MapParam
    Serial.printf("t: %f %f ofu: %f %f theta: %f\n", 
      current_time, Params::control_interval_sec, 
      SensorValue::optical_flow_vx, SensorValue::optical_flow_vy, 
      robot_pos.static_frame.rot.getAngle());
  }
}

} // namespace Task
