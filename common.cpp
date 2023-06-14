#include "common.h"
#include <algorithm>

uint8_t encode_float(float value, float max_value){
  float tmp = (value / max_value + 1.0f) * 127.0f;
  tmp = std::min(std::max(tmp, 0.0f), 254.0f);
  
  uint8_t ret = static_cast<uint8_t>(tmp);
  if(value == 0.0f){
    ret = 127;
  }
  return ret;
}

float decode_uint8(uint8_t value, float max_value){
  float tmp = value / 127.0f - 1.0f;

  float ret = tmp * max_value;
  if(value == 127){
    ret = 0.0f;
  }else if(value == 255){
    ret = 0.0f;
  }
  return ret;
}
