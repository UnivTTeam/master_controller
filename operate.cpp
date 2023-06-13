#include "map.h"
#include "device.h"
#include "params.h"

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
  CommandValue::wheel_vx = operate_dest.dynamic_frame[0].pos.x;
  CommandValue::wheel_vy = operate_dest.dynamic_frame[0].pos.y;
  CommandValue::wheel_vw = operate_dest.dynamic_frame[0].rot;
}
