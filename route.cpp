#include "route.h"
#include "params.h"
#include "task.h"
#include "map.h"
#include "elevator.h"

namespace Route {

using linear::Vec2, linear::Rot2;

// BangBang
BangBang::BangBang(float X_, float V_, float A_)
{
  X = X_; V = V_; A = A_;
  x = 0.0f; v = 0.0f; t = 0.0f;

  Xacc = V * V / A;
  if(Xacc > X){
    Xacc = X;
    Vmax = std::sqrt(A * X);
    Tvel = 0.0f;
  }else{
    Vmax = V;
    Tvel = (X - Xacc) / Vmax; 
  }

  Tacc = Vmax / A;
  Ttotal = Tacc + Tvel + Tacc;
}

void BangBang::setT(float t_)
{
  t = t_;
  if(t < Tacc){
    x = 0.5f * A * t * t;
    v = A * t;
  }else if(t < Tacc + Tvel){
    float dt = t - Tacc;
    x = 0.5f * Xacc + Vmax * dt;
    v = Vmax;
  }else if(t < Ttotal){
    float t_ = Ttotal - t;
    x = X - 0.5f * A * t_ * t_;
    v = A * t_;
  }else{
    x = x;
    v = 0.0f;
  }
};

float BangBang::getT(float x) const
{
  if(x < 0.0f){
    return 0.0f;
  }else if(x < 0.5f * Xacc){
    return std::sqrt(2.0f * max(x, 0.0f) / A);
  }else if(t < X - 0.5f * Xacc){
    float dx = x - 0.5f * Xacc;
    return Tacc + dx / A;
  }else if(t < X){
    float dx = X - x;
    return Ttotal - std::sqrt(2.0f * max(dx, 0.0f) / A);
  }else{
    return Ttotal;
  }
}

// RotRoute
RotRoute::RotRoute(float theta)
{
  t0 = Params::current_time;
  theta0 = Task::theta_dest;

  if(theta < 0.0f){
    dir = -1.0f;
  }else{
    dir = 1.0f;
  }
  bangbang = BangBang(
    theta * dir,
    Params::AUTO_CONTROL_ROT_VEL,
    Params::AUTO_CONTROL_ROT_ACC
  );

  Task::setAutoMode();
}

bool RotRoute::operator()() {
  bangbang.setT(Params::current_time - t0);
  Task::theta_dest = theta0 + dir * bangbang.getX();
  Task::omega_dest = dir * bangbang.getV();
  return bangbang.isEnd();
}

// ParaRoute
ParaRoute::ParaRoute(float x, float y){
  near_end = false;
  t0 = Params::current_time;
  r0 = robot_pos.static_frame.pos;

  dr = linear::Vec2<float>(x, y);
  ex = (1.0f / dr.norm()) * dr;
  ey = linear::Vec2<float>(-ex.y, ex.x);
  bangbang = BangBang(
    dr * ex,
    Params::AUTO_CONTROL_PARA_VEL,
    Params::AUTO_CONTROL_PARA_ACC
  );

  Task::setAutoMode();
}

bool ParaRoute::operator()(){
  linear::Vec2<float> r = robot_pos.static_frame.pos - r0;
  float x = r * ex;
  float y = r * ey;
  float t = max(bangbang.getT(x), 0.1f);

  if(!near_end){
    bangbang.setT(t);
    if(bangbang.isNearEnd()){
      near_end = true;
      t0 = Params::current_time - t;
    }
  }
  if(near_end){
    t = Params::current_time - t0;
  }
  bangbang.setT(t);

  float x_dest = bangbang.getX();
  float vx_ff = bangbang.getV();

  float vx = -Params::paraKp * (x-x_dest) + vx_ff;
  float vy = -Params::paraKp * y;
  Task::v_dest = (vx*ex) + (vy*ey);

  return Task::interruptAutoMode();
}

// ParaRotParaRoute
ParaRotParaRoute::ParaRotParaRoute(
    const Vec2<float>& dr0_, float theta_, const Vec2<float>& dr1_){
  t0 = Params::current_time;
  step = -1;
  need_elevetor_up = true;

  dr0 = dr0_;
  theta = theta_;  
  dr1 = dr1_;

  Task::setAutoMode();
}

bool ParaRotParaRoute::operator()(){
  if(step == -1){
    step++;
    para = ParaRoute(dr0.x, dr0.y);
  }
  if(step == 0){
    para();
    if(para.isEnd()){
      step++;
      rot = RotRoute(theta);
    }
  }
  if(step == 1){
    rot();
    if(rot.isEnd()){
      step++;
      para = ParaRoute(dr1.x, dr1.y);
      y0 = robot_pos.static_frame.pos.y;
    }
  }
  if(step == 2){
    para();
    float dy = abs(y0 - robot_pos.static_frame.pos.y);
    if(need_elevetor_up && dy > Params::ELEVATOR_UP_Y_DIFF){
      Elevator::setElevator();
      need_elevetor_up = false;
    }
    if(para.isEnd()){
      step++;
      para = ParaRoute(dr1.x, dr1.y);
      if(need_elevetor_up) {
        Elevator::setElevator();
        need_elevetor_up = false;
      }
    }
  }

  if(step == 2){
    return Task::interruptAutoMode();
  }
  return false;
}

// GTGTRoute
GTGTRoute::GTGTRoute(){
  step = -1;
  para = ParaRoute(0.0f, -700.0f);

  Task::setAutoMode();
}

bool GTGTRoute::operator()(){
  para();
  if(para.isEnd()){
    step++;
    if(step==0){
      para = ParaRoute(2000.0f, 0.0f);
    }else if(step%2 == 1){
      para = ParaRoute(-1000.0f, 0.0f);
    }else{
      para = ParaRoute(1000.0f, 0.0f);
    }
  }

  return false;
}

} // namespace Route
