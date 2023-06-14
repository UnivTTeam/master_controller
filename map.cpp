#include "map.h"
#include "params.h"
#include "device.h"

Transform::MultidiffTransform<float, 1> robot_pos;
 
void initMap()
{
  robot_pos = Transform::MultidiffTransform<float, 1>(Params::init_pos);
}

void updateMap(){
  // 角速度を求める
  static float last_imu_yaw = SensorValue::imu_yaw;
  float dTheta = std::remainder(SensorValue::imu_yaw - last_imu_yaw, 2 * M_PI);
  float omega = dTheta / Params::control_interval_sec;
  last_imu_yaw = SensorValue::imu_yaw;

  // フィールド座標系からオプティカルフロー座標系への変換を求める
  Transform::StaticTransform<float> optical_flow_static_frame = 
      robot_pos.staticTransform() + Params::optical_flow_pos;
  linear::Vec2<float> optical_flow_vel = 10.0f * linear::Vec2<float>(SensorValue::optical_flow_vx, SensorValue::optical_flow_vy);
  Transform::DynamicTransform<float> optical_flow_dynamic_frame0(optical_flow_static_frame.rot * optical_flow_vel, omega);
      
  Transform::MultidiffTransform<float, 1> optical_flow_frame(optical_flow_static_frame, optical_flow_dynamic_frame0);
  
  // フィールド座標系からマシン座標系への変換を求める
  robot_pos = optical_flow_frame + (-Params::optical_flow_pos);
  
  // 時間発展させる
  robot_pos.timeDevelopment(Params::control_interval_sec); 
}
