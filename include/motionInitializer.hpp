#pragma once

#include "cyclicSession.hpp"
#include "motors.hpp"

class AllMotors;

class IMotionInitializer {
 public:
  virtual ~IMotionInitializer() = default;

  virtual bool motor_initial_connect(const char* ifname, int motor_count, MotorModes mode) = 0;
  virtual bool run_async(CyclicSession& session, AllMotors& motors) = 0;
  virtual void motor_stop() = 0;
  virtual void motor_close() = 0;
};
