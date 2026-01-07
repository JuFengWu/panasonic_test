#include "fakeInitializer.hpp"

bool FakeInitializer::motor_initial_connect(const char* ifname, int motor_count, MotorModes mode)
{
  (void)ifname;
  (void)motor_count;
  (void)mode;
  opened_ = true;
  return true;
}

bool FakeInitializer::run_async()
{
  if (!opened_) return false;
  running_ = true;
  return true;
}

void FakeInitializer::stop() { running_ = false; }

void FakeInitializer::close()
{
  running_ = false;
  opened_ = false;
}
