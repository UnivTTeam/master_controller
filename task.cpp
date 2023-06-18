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
  MapParam = 2,
};
Mode mode = Mode::Manual;

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

std::function<bool()> auto_mode_callback = [](){ return true; };
void setAutoMode()
{
  mode = Mode::Auto;
}

int task_step = 0;
void autoTask()
{
  using Elevator::setElevator;
  if(task_step==0){
    auto_mode_callback = Route::GeneralRoute({{0.0f, 1550.0f}, {-900.0f, 1550.0f}}, 0, 0.0f);
  }else if(task_step<=2){
    auto_mode_callback = Route::ParaRoute(-2600.0f, 0.0f);
  }else if(task_step==3){
    auto_mode_callback = Route::GeneralRoute({{-700.0f, 0.0f}, {M_PI}, {0.0f, 4600.0f}}, 2);
  }else if(task_step<=5){
    auto_mode_callback = Route::ParaRoute(2600.0f, 0.0f);
  }else if(task_step==6){
    auto_mode_callback = Route::GeneralRoute({{600.0f, 0.0f}, {-M_PI}, {700.0f, 0.0f}, {0.0f, -2300.0f}}, 3);
  }else if(task_step<=9){
    auto_mode_callback = Route::ParaRoute(-2600.0f, 0.0f);
  }else{
    auto_mode_callback = Route::GTGTRoute();
  }
  task_step++;
}

void taskCallback() {
  // ボタン読み込み
  bool up = up_wrapper(PS4.Up());
  bool left = up_wrapper(PS4.Left());
  bool right = up_wrapper(PS4.Right());
  bool down = up_wrapper(PS4.Down());
  bool l1 = l1_wrapper(PS4.L1());
  bool r1 = r1_wrapper(PS4.R1());

  // モード読み込み
  if(mode != Mode::MapParam){
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
    if (l1) {
      float dtheta = Params::l1r1_rot_angle;
      robot_pos.static_frame.rot = Rot2<float>(robot_pos.static_frame.rot.getAngle() - dtheta);
      auto_mode_callback = Route::RotAdjustRoute();
    } else if (r1) {
      float dtheta = -Params::l1r1_rot_angle;
      robot_pos.static_frame.rot = Rot2<float>(robot_pos.static_frame.rot.getAngle() - dtheta);
      auto_mode_callback = Route::RotAdjustRoute();
    } else if (PS4.L2()) { // L2回転
      auto_mode_callback = Route::RotRoute(Params::AUTO_CONTROL_ROT_ANGLE);
    } else if (PS4.R2()) { // R2回転
      auto_mode_callback = Route::RotRoute(-Params::AUTO_CONTROL_ROT_ANGLE);
    } else if (PS4.Triangle()) {
      auto_mode_callback = Route::GTGTRoute();
    } else if (PS4.Circle()) {
      autoTask();
    } else if (PS4.Square()){
      auto_mode_callback = Route::GeneralRoute({
        {1000.0f, 0.0f}, {0.5f * M_PI}, 
        {0.0f, 1000.0f}, {0.5f * M_PI}, 
        {-1000.0f, 0.0f}, {-0.5f * M_PI}, 
        {0.0f, -1000.0f}, {-0.5f * M_PI}});
      // setAutoPara(1760.0f, 1760.0f);
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
    using namespace Elevator;
    if(right){
      resetElevator();
    }
    if(left){
      resetElevator();
      elevator_step--;
    }
    bool is_end = elevatorCallback();

    if(is_end & up){
      Serial.printf("熊手上昇指令 %d\n", elevator_step);
      setElevator();
    }
  } else if(mode == Mode::Emergency){
    Elevator::stopElevator();
  }

  // 足回り処理
  linear::Vec2<float> last_v_dest = v_dest;
  v_dest = Vec2<float>(0.0f, 0.0f);
  omega_dest = 0.0f;
  if(mode == Mode::Manual) {
    // スティック入力
    v_dest = Params::MANUAL_MAX_PARA_VEL * readStick();

    // 加速度制限
    linear::Vec2<float> acc = (v_dest - last_v_dest) / Params::control_interval_sec;
    if(acc.norm() > Params::AUTO_CONTROL_PARA_ACC){
      acc = (Params::AUTO_CONTROL_PARA_ACC / acc.norm()) * acc;
      v_dest = last_v_dest + acc * Params::control_interval_sec;
    }

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
