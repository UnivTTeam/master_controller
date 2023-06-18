#pragma once
#include <cmath>
#include <Arduino.h>

#include "libwheels/linear_algebra/linear_algebra.hpp"

namespace Route {

using linear::Vec2, linear::Rot2;

struct BangBang {
  BangBang(){}
  BangBang(float X_, float V_, float A_);

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
};

struct RotRoute{
  RotRoute(){}
  RotRoute(float theta);

  bool isEnd() { return bangbang.isEnd(); }
  bool operator()();

private:
  float t0, theta0;

  BangBang bangbang;
  float dir;
};

struct ParaRoute{
  ParaRoute(){}
  ParaRoute(float x, float y);

  bool isEnd() { return bangbang.isEnd(); }
  bool operator()();

private:
  bool near_end;
  float t0;
  Vec2<float> r0;

  BangBang bangbang;
  Vec2<float> dr, ex, ey;
};

struct ParaRotParaRoute{
  ParaRotParaRoute(const Vec2<float>& dr0_, float theta_, const Vec2<float>& dr1_);

  bool operator()();

private:
  float t0;
  int step;
  float y0;
  bool need_elevetor_up;

  Vec2<float> dr0;
  float theta;
  Vec2<float> dr1;
  
  RotRoute rot;
  ParaRoute para;  
};

struct GTGTRoute{
  GTGTRoute();

  bool operator()();

private:
  int step;

  RotRoute rot;
  ParaRoute para;  
};


} // namespace Route
