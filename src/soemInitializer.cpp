#include "soemInitializer.hpp"

bool SoemInitializer::motor_initial_connect(const char* ifname, int motor_count, MotorModel model, MotorModes mode)
{
  (void)ifname;
  (void)motor_count;
  (void)model;
  (void)mode;
  opened_ = true;
  initialized_ = true;
  return true;
}

bool SoemInitializer::run_async()
{
  if (!opened_) return false;
  running_ = true;
  return true;
}

void SoemInitializer::stop() { running_ = false; }

void SoemInitializer::close()
{
  running_ = false;
  opened_ = false;
}
