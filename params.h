#pragma once

#include "libwheels/transform2d/static_transform.hpp"

// 制御パラメタを入れるファイル
namespace Params{

// 制御周期: 20ms
inline const float control_interval_us = 20000.0f;
inline const float control_interval_sec = control_interval_us / 1000000.f;

// フィールド上の初期位置情報(x, y, theta)
inline const Transform::StaticTransform<float> init_pos(0.0f, 0.0f, M_PI / 2);

// マシン座標上のオプティカルフロー位置(x, y, theta)
inline const Transform::StaticTransform<float> optical_flow_pos(0.0f, 0.0f, M_PI / 2);

// 制御パラメタ
inline const float rotKp = 4.0;
}

