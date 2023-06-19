#pragma once
#include <cmath>
#include <vector>
#include <Arduino.h>

#include "libwheels/linear_algebra/linear_algebra.hpp"
#include "params.h"

namespace Route {

using linear::Vec2, linear::Rot2;

// 経路の実態
// 直接呼ぶことはない
struct BangBang {
  BangBang(){}
  BangBang(float X_, float V_, float A_, float time_mergin_=0.0f);

  void setT(float t_);
  float getT(float x) const;
  bool isEnd() { return t >= Ttotal + time_mergin; }
  bool isNearEnd() { return t >= Ttotal - Tacc; }

  float getX() { return x; }
  float getV() { return v; }

private:
  float x, v, t;
  float X, V, A;
  float Xacc, Vmax, Tacc, Tvel, Ttotal;
  float time_mergin;
};

// 回転経路，与えられた角度だけ回転する
struct RotRoute{
  RotRoute(){}
  RotRoute(float theta, float time_mergin = 0.0f);

  bool isEnd();
  bool operator()();

private:
  bool near_end;
  float t0, theta0;

  BangBang bangbang;
  float dir;
};

// 回転経路，自己位置認識に基づいて回転方向を揃える
struct RotAdjustRoute{
  RotAdjustRoute();

  bool isEnd() { return route.isEnd(); }
  bool operator()();

private:
  RotRoute route;
};

// 並進経路
struct ParaRoute{
  ParaRoute(){}
  ParaRoute(float x, float y, float time_mergin = 0.0f);

  bool isEnd();
  float getX() { return bangbang.getX(); }
  bool operator()();

private:
  bool near_end;
  float t0;
  Vec2<float> r0;

  BangBang bangbang;
  Vec2<float> dr, ex, ey;
};

// 一般経路
// 回転経路，並進経路を順番に実施
//   dataのところが2次元vectorなら並進，1次元vectorなら回転
// 途中で上昇機構を上げる場合，何番目(0-index)の経路で上げるか指定
// また，移動開始直後に上げると干渉する場合があるので，elevator_move_length だけ移動してから上げる
struct GeneralRoute {
  GeneralRoute(
    std::vector<std::vector<float>> data_,
    int elevator_step_=-1,  // デフォルトは上昇機構なし
    float elevator_move_length_=Params::ELEVATOR_UP_Y_DIFF,
    float time_mergin_=0.0f);

  bool setNewRoute();
  bool operator()();

private:
  int step;
  int elevator_step;
  float time_mergin;
  int max_step;

  std::vector<std::vector<float>> data;
  float elevator_move_length;

  bool is_para_route;
  RotRoute rot;
  ParaRoute para;
};

// 最後のガタガタルート
struct GTGTRoute{
  GTGTRoute();

  bool operator()();

private:
  int step;

  RotRoute rot;
  ParaRoute para;  
};


} // namespace Route
