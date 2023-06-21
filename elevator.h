#pragma once

namespace Elevator{

extern volatile int elevator_step;

void elevatorCallback();

void resetElevator();
void retryElevator();
void retryElevator(int i);
void stopElevator();

} // namespace Elevator
