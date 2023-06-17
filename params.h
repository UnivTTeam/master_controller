#pragma once

#include <vector>

#include "libwheels/transform2d/static_transform.hpp"

// 制御パラメタを入れるファイル
namespace Params{

// 制御周期: 20ms
inline constexpr float CONTROL_INTERVAL_MS = 20.0f;
inline float control_interval_sec = CONTROL_INTERVAL_MS / 1000.f;

// フィールド上の初期位置情報(x, y, theta)
inline const Transform::StaticTransform<float> init_pos(0.0f, 0.0f, 0.0f * M_PI);

// マシン座標上のオプティカルフロー位置(x, y, theta)
inline const Transform::StaticTransform<float> optical_flow_pos(-60.0f, 250.0f, 0.04425343466456355f);

// 制御パラメタ
inline constexpr float rotKp = 4.0f;
inline constexpr float l1r1_rot_angle = (3.0f/180.0f) * M_PI;
inline constexpr float l2r2_rot_angle = 0.5f * M_PI;
inline constexpr float l2r2_rot_time = 1.0f;

inline constexpr float imu_calibration_time_sec = 5.0f;
inline constexpr float imu_calibration_start_time_sec = 3.0f;

inline constexpr float MAX_PARA_VEL = 1000.0f;

// ジャイロの生の値を角度に変更する定数 [rad / rawZ]
inline constexpr float gyro_scale = 4.05f; // [4.048290243592277, 4.0505555454999325, 4.052528003206573, 4.035715515291389]
inline constexpr float optical_flow_scale = 47.9230059159935f; // 90.05186170389779f;

// ニクロム線のピン番号
inline const std::vector<int> ELEVATOR_PIN = {{13,14,27,26,25}};
inline constexpr float ELEVATOR_TIME = 2.5f;

// LED
inline constexpr int RED_LED = 35;
inline constexpr int YELLOW_LED = 34;
inline constexpr int GREEN_LED = 32;
}

