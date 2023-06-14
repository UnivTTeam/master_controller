#pragma once

#include "libwheels/transform2d/static_transform.hpp"

// 制御パラメタを入れるファイル
namespace Params{

// 制御周期: 20ms
inline const float control_interval_ms = 20.0f;
inline const float control_interval_us = control_interval_ms * 1000.0f;
inline const float control_interval_sec = control_interval_ms / 1000.f;

// フィールド上の初期位置情報(x, y, theta)
inline const Transform::StaticTransform<float> init_pos(0.0f, 0.0f, 0.0f * M_PI);

// マシン座標上のオプティカルフロー位置(x, y, theta)
inline const Transform::StaticTransform<float> optical_flow_pos(0.0f, 0.0f, 0.0f);

// 制御パラメタ
inline const float rotKp = 4.0f;
inline const float l1r1_rot_angle = (1.0f/180.0f) * M_PI;
inline const float l2r2_rot_angle = 0.5f * M_PI;
inline const float l2r2_rot_time = 1.0f;

// 通信パラメタ：親機と子機で同期する
inline const float MAX_PARA_VEL = 5.0f;
inline const float MAX_ROT_VEL = 5.0f;

inline const float imu_calibration_time_sec = 5.0f;
inline const float imu_calibration_start_time_sec = 2.0f;

// ジャイロの生の値を角度に変更する定数 [rad / rawZ]
inline const float gyro_scale = 4.062495349365322f;
}

