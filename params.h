#pragma once

#include <vector>

#include "libwheels/transform2d/static_transform.hpp"

// 制御パラメタを入れるファイル
namespace Params{

// 制御周期: 20ms
inline constexpr float CONTROL_INTERVAL_MS = 20.0f;
inline float control_interval_sec = CONTROL_INTERVAL_MS / 1000.f;
inline float current_time = 0.0f;

// フィールド上の初期位置情報(x, y, theta)
inline const Transform::StaticTransform<float> init_pos(0.0f, 0.0f, M_PI);

// マシン座標上のオプティカルフロー位置(x, y, theta)
inline const Transform::StaticTransform<float> optical_flow_pos(-60.0f, 250.0f, 0.039716020318083745f);

// 制御パラメタ
inline constexpr float l1r1_rot_angle = (3.0f/180.0f) * M_PI;

inline constexpr float imu_calibration_time_sec = 8.0f;
inline constexpr float imu_calibration_start_time_sec = 4.0f;

inline constexpr float MANUAL_MAX_PARA_VEL = 1600.0f;

inline constexpr float AUTO_CONTROL_PARA_VEL = 2000.0f;
inline constexpr float AUTO_CONTROL_PARA_ACC = 2000.0f;

inline constexpr float AUTO_CONTROL_ROT_ANGLE = 0.5f * M_PI;
inline constexpr float AUTO_CONTROL_ROT_VEL = 2.0f * M_PI;  // v=1500 <--> omega=3.5
inline constexpr float AUTO_CONTROL_ROT_ACC = 1.5f * M_PI;  // a=2000 <--> alpha=2.0

inline constexpr float AUTO_MODE_MANUAL_POS_CHANGE = 300.0f / 0.5f;

inline constexpr float MAX_ROT_VEL = 3.0f * M_PI;

inline constexpr float rotKp = 5.0f;
inline constexpr float paraKp = 5.0f;

inline constexpr float AUTO_CONTROL_PARA_STOP_VEL = 50.0f;
inline constexpr float AUTO_CONTROL_ROT_STOP_VEL = 0.1f;

// ジャイロの生の値を角度に変更する定数 [rad / rawZ]
inline constexpr float gyro_scale = 4.05f; // [4.048290243592277, 4.0505555454999325, 4.052528003206573, 4.035715515291389]
inline constexpr float optical_flow_scale = 42.2f; // 90.05186170389779f;

// ニクロム線のピン番号
inline const std::vector<std::vector<int>> ELEVATOR_PINS = {{{13,25},{14},{27,26}}};
inline constexpr float ELEVATOR_TIME = 0.2f;
inline constexpr float ELEVATOR_UP_Y_DIFF = 700.0f; // _mm

// LED
inline constexpr int RED_LED = 35;
inline constexpr int YELLOW_LED = 34;
inline constexpr int GREEN_LED = 32;
}
