#include "map.h"
#include "device.h"
#include "params.h"

float scaling(float value, float max_value)
{
    if(value > max_value){
      value = max_value;
    }else if (value < -max_value){
      value = -max_value;
    }
    return value / max_value;
}

void setVelocityFromField(float vx0, float vy0, float theta0)
{
  // thetaのP制御
  float dtheta = std::remainder(robot_pos.static_frame.rot.getAngle() - theta0, 2 * M_PI);
  float omega0 = -Params::rotKp * dtheta;

  // マシン座標的目標速度の取得
  Transform::MultidiffTransform<float, 1> robot_dest(
    robot_pos.staticTransform(),
    Transform::DynamicTransform<float>(vx0, vy0, omega0)
  );
  Transform::MultidiffTransform<float, 1> operate_dest = (-robot_pos.staticTransform()) + robot_dest;

  // 指令値を書き込み
  CommandValue::wheel_vx = scaling(operate_dest.dynamic_frame[0].pos.x, Params::MAX_PARA_VEL);
  CommandValue::wheel_vy = scaling(operate_dest.dynamic_frame[0].pos.y, Params::MAX_PARA_VEL);
  CommandValue::wheel_vw = scaling(operate_dest.dynamic_frame[0].rot, Params::MAX_ROT_VEL);
}
