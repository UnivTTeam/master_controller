#pragma once
#include "libwheels/transform2d/transform2d.hpp"

extern Transform::MultidiffTransform<float, 1> robot_pos;

void initMap();
void updateMap();
