#include <algorithm>

#include "map.h"
#include "device.h"
#include "params.h"
#include "task.h"

float scaling(float value, float max_value)
{
    if(value > max_value){
      value = max_value;
    }else if (value < -max_value){
      value = -max_value;
    }
    return value / max_value;
}

void setVelocityFromField()
{
  const float vx0 = Task::v_dest.x;
  const float vy0 = Task::v_dest.y;
  const float theta0 = Task::theta_dest;
  const float omega_ff = Task::omega_dest;

  // thetaのP制御
  float dtheta = std::remainder(robot_pos.static_frame.rot.getAngle() - theta0, 2 * M_PI);
  float omega0 = -Params::rotKp * dtheta + omega_ff;
  if(omega0 > Params::MAX_ROT_VEL){
    omega0 = Params::MAX_ROT_VEL;
  }else if(omega0 < -Params::MAX_ROT_VEL){
    omega0 = -Params::MAX_ROT_VEL;
  }

  // マシン座標的目標速度の取得
  Transform::MultidiffTransform<float, 1> robot_dest(
    robot_pos.staticTransform(),
    Transform::DynamicTransform<float>(vx0, vy0, omega0)
  );
  Transform::MultidiffTransform<float, 1> operate_dest = (-robot_pos.staticTransform()) + robot_dest;

  // 指令値を書き込み
  CommandValue::wheel_vx = operate_dest.dynamic_frame[0].pos.x;
  CommandValue::wheel_vy = operate_dest.dynamic_frame[0].pos.y;
  CommandValue::wheel_vw = operate_dest.dynamic_frame[0].rot;
}
