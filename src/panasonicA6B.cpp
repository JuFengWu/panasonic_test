#include "panasonicA6B.hpp"

Motor::Motor(int slave) { (void)slave; }

PanasonicA6B::PanasonicA6B(int slave)
    : Motor(slave), slave_(slave) {}

bool PanasonicA6B::set_mode() { return false; }

bool PanasonicA6B::set_target_position(float target)
{
  (void)target;
  return false;
}

bool PanasonicA6B::set_target_velocity(float target)
{
  (void)target;
  return false;
}

bool PanasonicA6B::set_target_torque(float target)
{
  (void)target;
  return false;
}

int PanasonicA6B::get_error_code() { return 0; }

int PanasonicA6B::get_current_position() { return 0; }

int PanasonicA6B::get_current_velocity() { return 0; }

int PanasonicA6B::get_current_torque() { return 0; }
