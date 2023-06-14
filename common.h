#pragma once

#include <cstdint>

namespace Params{
// 通信パラメタ：親機と子機で同期する
inline const float MAX_PARA_VEL = 1000.0f;  // [mm / sec]
inline const float MAX_ROT_VEL = 10.0f;     // [rad / sec]
}

uint8_t encode_float(float value, float max_value);
float decode_uint8(uint8_t value, float max_value);
