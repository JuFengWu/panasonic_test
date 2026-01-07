#pragma once

#include "motors.hpp"

class IMotionInitializer {
 public:
  virtual ~IMotionInitializer() = default;

  virtual bool motor_initial_connect(const char* ifname, int motor_count, MotorModel model, MotorModes mode) = 0;
  virtual bool run_async() = 0;
  virtual void stop() = 0;
  virtual void close() = 0;
};
