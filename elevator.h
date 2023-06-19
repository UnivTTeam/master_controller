#pragma once

namespace Elevator{

extern volatile int elevator_step;

bool elevatorCallback();

void setElevator();
void resetElevator();
void retryElevator();
void stopElevator();

} // namespace Elevator
