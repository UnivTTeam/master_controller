#include <Arduino.h>
#include "elevator.h"
#include "params.h"

namespace Elevator{

using Params::current_time;

int target_pin = 0;
float start_time = -100000000.0f;
volatile int elevator_step = 0;

void elevatorCallback()
{
  if(current_time < end_time){
    for(int i=0; i<Params::ELEVATOR_PINS.size(); i++){
      const auto pins = Params::ELEVATOR_PINS[i];
      float t = current_time - start_time;
      for(int j=0; j<pins.size(); j++){
        int k = t / Params::ELEVATOR_TIME;
        digitalWrite(pins[j], (i==target_pin)&(j==k));
      }
    }
  }
}

void resetElevator()
{
  start_time = -100000000000.0f;
}

void retryElevator()
{
  start_time = current_time;
}

void retryElevator(int i)
{
  target_pin = i;
  start_time = current_time;
}

void stopElevator()
{
  resetElevator();
  for(const auto& pins: Params::ELEVATOR_PINS){
    for(const auto& pin : pins) {
      digitalWrite(pin, LOW);
    }
  }
}
} // namespace Elevator
