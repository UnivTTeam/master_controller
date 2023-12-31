#pragma once
#include <cmath>
#include <vector>
#include <Arduino.h>

#include "libwheels/linear_algebra/linear_algebra.hpp"
#include "params.h"

namespace Route {

using linear::Vec2, linear::Rot2;

constexpr float ROT_TIME_MERGIN = 0.1f;
constexpr float PARA_TIME_MERGIN = 0.0f;

// 経路の実態
// 直接呼ぶことはない
struct BangBang {
  BangBang(){}
  BangBang(float X_, float V_, float A_, float time_mergin_=0.0f);

  void setT(float t_);
  float getT(float x) const;
  bool isEnd() { return t >= Ttotal; }
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
  RotRoute(float theta, float time_mergin = ROT_TIME_MERGIN);

  bool isEnd();
  bool operator()();

private:
  bool near_end;
  float t0, theta0;

  BangBang bangbang;
  float dir;
};

// 並進経路
struct ParaRoute{
  ParaRoute(){}
  ParaRoute(float x, float y, float time_mergin = PARA_TIME_MERGIN);

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

// 並進経路
struct LinearRoute{
//  ParaRoute(){}
  LinearRoute(float x, float y);
  
  bool isEnd(){ return false; }
  bool operator()();

private:
  float t0;
  Vec2<float> r0;

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
    int elevator_target_=-1,
    int elevator_step_=-1,  // デフォルトは上昇機構なし
    float elevator_move_length_=Params::ELEVATOR_UP_Y_DIFF);

  bool setNewRoute();
  bool operator()();

private:
  void callRoute();
  bool routeIsEnd();

  int step;
  int elevator_step;
  int elevator_target;
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
  void setNewRoute(float x, float y);
  
  int step;
  float t0;
  BangBang bangbang;
  Vec2<float> e;
};

void addRdiff(const Vec2<float>& x);

} // namespace Route
