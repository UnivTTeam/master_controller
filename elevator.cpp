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
  float t = current_time - start_time;

  if(target_pin == 0){
    if(t < Params::ELEVATOR_TIME){
      digitalWrite(Params::ELEVATOR_PIN0, HIGH);
    }else{
      digitalWrite(Params::ELEVATOR_PIN0, LOW);
    }
    if(t < Params::FUTA_TIME){
      digitalWrite(Params::FUTA_PIN, HIGH);
    }else{
      digitalWrite(Params::FUTA_PIN, LOW);
    }
    digitalWrite(Params::ELEVATOR_PIN1, LOW);
    digitalWrite(Params::ELEVATOR_PIN2A, LOW);
    digitalWrite(Params::ELEVATOR_PIN2B, LOW);
  } else if(target_pin == 1){
    if(t < Params::ELEVATOR_TIME){
      digitalWrite(Params::ELEVATOR_PIN1, HIGH);
      digitalWrite(Params::FUTA_PIN, HIGH);
    }else{
      digitalWrite(Params::ELEVATOR_PIN1, LOW);
      digitalWrite(Params::FUTA_PIN, LOW);
    }
    digitalWrite(Params::ELEVATOR_PIN0, LOW);
    digitalWrite(Params::ELEVATOR_PIN2A, LOW);
    digitalWrite(Params::ELEVATOR_PIN2B, LOW);
  }else if(target_pin == 2){
    if(t < Params::ELEVATOR_TIME){
      digitalWrite(Params::ELEVATOR_PIN2A, HIGH);
      digitalWrite(Params::ELEVATOR_PIN2B, LOW);
      digitalWrite(Params::FUTA_PIN, HIGH);
    }else if(t < 2.0f*Params::ELEVATOR_TIME){
      digitalWrite(Params::ELEVATOR_PIN2A, LOW);
      digitalWrite(Params::ELEVATOR_PIN2B, HIGH);
      digitalWrite(Params::FUTA_PIN, LOW);
    }else{
      digitalWrite(Params::ELEVATOR_PIN2A, LOW);
      digitalWrite(Params::ELEVATOR_PIN2B, LOW);
      digitalWrite(Params::FUTA_PIN, LOW);
    }
    digitalWrite(Params::ELEVATOR_PIN0, LOW);
    digitalWrite(Params::ELEVATOR_PIN1, LOW);
  }else{
    for(int pin : Params::ELEVATOR_PIN_LIST) {
      digitalWrite(pin, LOW);
    }
  }
}

void resetElevator()
{
  start_time = -100000000000.0f;
}

void retryElevator(int i)
{
  if(target_pin != i){
    resetElevator(); 
  }
  target_pin = i;

  float end_time = start_time;
  if(target_pin == 0){
    end_time += Params::ELEVATOR_TIME;
  } else if(target_pin == 1){
    end_time += Params::ELEVATOR_TIME;
  }else if(target_pin == 2){
    end_time += 2 * Params::ELEVATOR_TIME;
  }
  if(end_time < current_time){
    start_time = current_time;
  }
}

void stopElevator()
{
  resetElevator();
  for(int pin : Params::ELEVATOR_PIN_LIST) {
    digitalWrite(pin, LOW);
  }
}
} // namespace Elevator
