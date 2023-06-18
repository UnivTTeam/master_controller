#pragma once

namespace Elevator{

extern volatile int elevator_step;

bool elevatorCallback();

void setElevator();
void resetElevator();
void stopElevator();

} // namespace Elevator
