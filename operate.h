#pragma once
// vx0, vy0: フィールド座標的目標マシン速度
// theta: フィールド座標的マシン角度
void setVelocityFromField(float vx0, float vy0, float theta0, float omega_ff = 0.0f);