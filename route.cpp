#include "route.h"
#include "params.h"
#include "task.h"
#include "map.h"
#include "elevator.h"

namespace Route {

using linear::Vec2, linear::Rot2;

constexpr float minimum_route_time = 0.5f;

Vec2<float> r_diff(0.0f, 0.0f);

// BangBang
BangBang::BangBang(float X_, float V_, float A_, float time_mergin_)
{
  X = X_; V = V_; A = A_; time_mergin = time_mergin_;
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
RotRoute::RotRoute(float theta, float time_mergin)
{
  near_end = false;
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
    Params::AUTO_CONTROL_ROT_ACC,
    time_mergin
  );

  Task::setAutoMode();
}

bool RotRoute::isEnd()
{
  return (bangbang.isEnd() & (abs(robot_pos.dynamic_frame[0].rot) < Params::AUTO_CONTROL_PARA_STOP_VEL));
}

bool RotRoute::operator()() {
  float theta = (robot_pos.static_frame.rot.getAngle() - theta0) * dir;
  float t = max(bangbang.getT(theta), minimum_route_time);
  Serial.printf("RotRoute\n");

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
ParaRoute::ParaRoute(float x, float y, float time_mergin){
  Serial.printf("ParaRoute\n");
  near_end = false;
  t0 = Params::current_time;
  r0 = robot_pos.static_frame.pos;

  r_diff = Vec2<float>(0.0f, 0.0f);

  dr = linear::Vec2<float>(x, y);
  ex = (1.0f / dr.norm()) * dr;
  ey = linear::Vec2<float>(-ex.y, ex.x);
  bangbang = BangBang(
    dr * ex,
    Params::AUTO_CONTROL_PARA_VEL,
    Params::AUTO_CONTROL_PARA_ACC,
    time_mergin
  );

  Task::setAutoMode();
}

bool ParaRoute::isEnd()
{
  return (bangbang.isEnd() & (robot_pos.dynamic_frame[0].pos.norm() < Params::AUTO_CONTROL_PARA_STOP_VEL));
}

bool ParaRoute::operator()(){
  linear::Vec2<float> r = robot_pos.static_frame.pos - (r0+r_diff);
  float x = r * ex;
  float y = r * ey;
  float t = max(bangbang.getT(x), minimum_route_time);

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

void addRdiff(const Vec2<float>& x)
{
  r_diff = r_diff + x;
}

// GeneralRoute
GeneralRoute::GeneralRoute(
    std::vector<std::vector<float>> data_,
    int elevator_step_,
    float elevator_move_length_,
    float time_mergin_)
{
  Serial.printf("GeneralRoute\n");
  data = data_;
  elevator_move_length = elevator_move_length_;
  time_mergin = time_mergin_;
  max_step = data_.size();

  step = -1;
  elevator_step = elevator_step_;
  if(elevator_step == -1){
    elevator_step = data.size() + 100;
  }

  setNewRoute();

  Task::setAutoMode();
}

bool GeneralRoute::setNewRoute()
{
  if(step >= max_step){
    step = data.size();
    return false;
  }
  Serial.printf("setNewRoute %d\n", step);

  step++;
  Serial.printf("hoge %d\n", step);
  for(const auto& v: data){
    Serial.printf(" v.size()=%d\n", v.size());
  }
  while(step < max_step){
    std::vector<float> info = data[step];
    Serial.printf("info.size()=%d\n", info.size());
    if(info.size() == 2){
      is_para_route = true;
      para = ParaRoute(info[0], info[1], time_mergin);
      return true;
    } else if(info.size() == 1){
      is_para_route = false;
      rot = RotRoute(info[0], time_mergin);
      return true;
    } else {
      Serial.printf("invalid route\n");
    }
    step++;
  }
  return false;
}

void GeneralRoute::callRoute()
{
  if(is_para_route){
    para();
  }else{
    rot();
  }
}

bool GeneralRoute::routeIsEnd()
{
  if(is_para_route){
    return para.isEnd();
  }else{
    return rot.isEnd();
  }  
}

bool GeneralRoute::operator()(){
  bool is_end = true;
  while(is_end){
    callRoute();
    is_end = routeIsEnd();
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

  if(step == max_step -1 && is_para_route){
    return Task::interruptAutoMode();
  } else if(step == data.size()){
    return true;
  }
  return false;
}

// GTGTRoute
GTGTRoute::GTGTRoute(){
  step = 0;
  setNewRoute(0.0f, -700.0f);

  Task::setAutoMode();
}

void GTGTRoute::setNewRoute(float x, float y)
{
  auto r = Vec2<float>(x, y);

  t0 = Params::current_time;
  bangbang = BangBang(
    r.norm(),
    Params::AUTO_CONTROL_PARA_VEL,
    Params::AUTO_CONTROL_PARA_ACC
  );
  e = (1.0f / r.norm()) * r;
}

bool GTGTRoute::operator()(){
  float t = Params::current_time - t0;
  bangbang.setT(t);
  
  if(bangbang.isEnd()){
    if(step == 0){
      setNewRoute(1000.0f, 0.0f);
    }else if(step==1){
      setNewRoute(0.0f, 600.0f);
    }else if(step==2){
      setNewRoute(-500.0f, 0.0f);
    }else if(step==3){
      setNewRoute(500.0f, 0.0f);
    }else if(step==2){
      setNewRoute(0.0f, -500.0f);
    }else if(step==3){
      setNewRoute(0.0f, 500.0f);
    }else if(step==4){
      setNewRoute(-500.0f, 0.0f);
    }else if(step==5){
      setNewRoute(500.0f, 0.0f);
    }else if(step==6){
      setNewRoute(0.0f, -500.0f);
    }else if(step==7){
      setNewRoute(0.0f, 500.0f);
    }else{
      Task::callForceEmergency();
    }
    step++;
    bangbang.setT(0.0f);
  }
  Task::v_dest = bangbang.getV() * e;

  return false;
}

} // namespace Route
