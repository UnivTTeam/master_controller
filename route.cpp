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

// RotAdjustRoute
RotAdjustRoute::RotAdjustRoute() {
  float theta_dest = Task::theta_dest;
  Task::theta_dest = robot_pos.static_frame.rot.getAngle();
  route = RotRoute(theta_dest - Task::theta_dest);
}

bool RotAdjustRoute::operator()() {
  return route();
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

// GeneralRoute
GeneralRoute::GeneralRoute(
    const std::vector<std::vector<float>>& data_,
    int elevator_step_,
    float elevator_move_length_)
{
  data = data_;
  elevator_move_length = elevator_move_length_;

  step = -1;
  elevator_step = elevator_step_;
  if(elevator_step == -1){
    elevator_step = data.size() + 100;
  }

  setNewRoute();
}

bool GeneralRoute::setNewRoute()
{
  if(step >= data.size()){
    step = data.size();
    return false;
  }

  step++;
  while(step < data.size()){
    std::vector<float> info = data[step];
    if(info.size() == 2){
      is_para_route = true;
      para = ParaRoute(info[0], info[1]);
      return true;
    } else if(info.size() == 1){
      is_para_route = false;
      rot = RotRoute(info[0]);
      return true;
    }
    step++;
  }
  return false;
}

bool GeneralRoute::operator()(){
  bool is_end = true;
  while(is_end){
    if(is_para_route){
      para();
      is_end = para.isEnd();
    }else{
      rot();
      is_end = rot.isEnd();
    }
    if(is_end){
      // 新ルートの取得に失敗したらis_endにfalseが入る
      is_end = setNewRoute();
    }
  }

  if(step >= elevator_step){
    bool ok = true;
    if(is_para_route){
      ok = (para.getX() >= elevator_move_length);
    }
    if(ok){
      Elevator::setElevator();
      elevator_step = data.size() + 100;
    }
  }

  if(step == data.size() -1 && is_para_route){
    return Task::interruptAutoMode();
  } else if(step == data.size()){
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
