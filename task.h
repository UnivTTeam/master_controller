#pragma once
#include "libwheels/linear_algebra/linear_algebra.hpp"
#include "params.h"

namespace Task{
inline linear::Vec2<float> v_dest(0.0f, 0.0f);
inline float theta_dest = Params::init_pos.rot.getAngle();
inline float omega_dest = 0.0f;

bool interruptAutoMode();
void setAutoMode();
void callForceEmergency();

void setupTask();
void taskCallback();
}
