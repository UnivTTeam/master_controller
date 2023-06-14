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
  float omega = Params::gyro_scale * (SensorValue::imu_yaw - last_imu_yaw) / Params::control_interval_sec;
  last_imu_yaw = SensorValue::imu_yaw;

  // フィールド座標系からオプティカルフロー座標系への変換を求める
  Transform::StaticTransform<float> optical_flow_pos = 
      robot_pos.staticTransform() + Params::optical_flow_pos;
  Transform::DynamicTransform<float> optical_flow_vel(
      optical_flow_pos.rot * 
        linear::Vec2<float>(
          SensorValue::optical_flow_vx, 
          SensorValue::optical_flow_vy),
      omega);
      
  Transform::MultidiffTransform<float, 1> optical_flow_frame(optical_flow_pos, optical_flow_vel);
  
  // フィールド座標系からマシン座標系への変換を求める
  robot_pos = optical_flow_frame + (-Params::optical_flow_pos);
  
  // 時間発展させる
  robot_pos.timeDevelopment(Params::control_interval_sec); 
}
