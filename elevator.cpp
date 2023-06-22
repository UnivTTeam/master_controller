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
  Serial.printf("%d\n", Params::ELEVATOR_PINS_LISTS.size());
  for(int i=0; i<Params::ELEVATOR_PINS_LISTS.size(); i++){
    const auto& pins_list = Params::ELEVATOR_PINS_LISTS[i];
    if(i==target_pin){
      float t = current_time - start_time;
      int k = t / Params::ELEVATOR_TIME;
      for(int j=0; j<pins_list.size(); j++){
        for(const auto& pin : pins_list[j]){
          digitalWrite(pin, j==k);
        }
      }
    }else{
      for(const auto& pins : pins_list) {
        for(const auto& pin : pins) {
          digitalWrite(pin, LOW);
        }
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
  for(const auto& pins_list : Params::ELEVATOR_PINS_LISTS) {
    for(const auto& pins : pins_list) {
      for(const auto& pin : pins) {
        digitalWrite(pin, LOW);
      }
    }
  }
}
} // namespace Elevator
