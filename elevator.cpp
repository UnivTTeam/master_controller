#include <Arduino.h>
#include "elevator.h"
#include "params.h"

namespace Elevator{

using Params::current_time;

int target_pin = 0;
float end_time = -1.0f;
volatile int elevator_step = 0;

bool elevatorCallback()
{
  if(current_time < end_time){
    for(int i=0; i<Params::ELEVATOR_PINS.size(); i++){
      for(const auto& pin : Params::ELEVATOR_PINS[i]) {
        digitalWrite(pin, (i==target_pin));
      }
    }
    return false;
  }else{
    for(const auto& pins: Params::ELEVATOR_PINS){
      for(const auto& pin : pins) {
        digitalWrite(pin, LOW);
      }
    }
    return true;
  }
}

void setElevator()
{
  target_pin = elevator_step;
  end_time = current_time + Params::ELEVATOR_TIME;
  elevator_step++;
}

void resetElevator()
{
  end_time = -1.0f;
}

void retryElevator()
{
  end_time = current_time + Params::ELEVATOR_TIME;
}

void stopElevator()
{
  end_time = 0.0f;
  for(const auto& pins: Params::ELEVATOR_PINS){
    for(const auto& pin : pins) {
      digitalWrite(pin, LOW);
    }
  }
}
} // namespace Elevator
