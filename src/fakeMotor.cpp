#include "fakeMotor.hpp"

FakeMotor::FakeMotor(int slave, MotorModes mode)
    : Motor(slave, mode),
      current_position_(0),
      current_velocity_(0),
      current_torque_(0),
      error_code_(0),
      profile_{0, 0, 0},
      target_position_(0.0f),
      target_velocity_(0.0f),
      target_torque_(0.0f) {}

bool FakeMotor::set_mode(MotorModes mode)
{
  mode_ = mode;
  return true;
}

bool FakeMotor::set_target_position(float target)
{
  target_position_ = target;
  current_position_ = static_cast<int>(target);
  return true;
}

bool FakeMotor::set_target_velocity(float target)
{
  target_velocity_ = target;
  current_velocity_ = static_cast<int>(target);
  return true;
}

bool FakeMotor::set_target_torque(float target)
{
  target_torque_ = target;
  current_torque_ = static_cast<int>(target);
  return true;
}

bool FakeMotor::setMotionProfile(const MotionProfile& p)
{
  return setVel(p.vel) && setAcc(p.acc) && setDec(p.dec);
}

bool FakeMotor::setVel(int vel)
{
  profile_.vel = vel;
  return true;
}

bool FakeMotor::setAcc(int acc)
{
  profile_.acc = acc;
  return true;
}

bool FakeMotor::setDec(int dec)
{
  profile_.dec = dec;
  return true;
}

bool FakeMotor::servo_on()
{
  return true;
}

bool FakeMotor::servo_off()
{
  return true;
}

int FakeMotor::get_error_code() { return error_code_; }

double FakeMotor::get_current_position() { return current_position_; }

int FakeMotor::get_current_velocity() { return current_velocity_; }

int FakeMotor::get_current_torque() { return current_torque_; }

MotorModes FakeMotor::get_mode() { return mode_; }
