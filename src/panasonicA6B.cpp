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

bool PanasonicA6B::setMotionProfile(const MotionProfile& p)
{
  return setVel(p.vel) && setAcc(p.acc) && setDec(p.dec);
}

bool PanasonicA6B::setVel(int vel)
{
  (void)vel;
  return false;
}

bool PanasonicA6B::setAcc(int acc)
{
  (void)acc;
  return false;
}

bool PanasonicA6B::setDec(int dec)
{
  (void)dec;
  return false;
}

int PanasonicA6B::get_error_code() { return 0; }

int PanasonicA6B::get_current_position() { return 0; }

int PanasonicA6B::get_current_velocity() { return 0; }

int PanasonicA6B::get_current_torque() { return 0; }
