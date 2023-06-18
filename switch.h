#pragma once
#include <arduino.h>

struct SwitchUpperTrigger {
  SwitchUpperTrigger() : last_state(true)
  {
  }

  bool operator()(bool state){
    bool trigger = state ^ last_state;
    last_state = state;

    return (trigger & state);
  }

private:
  bool last_state;
  int last_time;
};

