#pragma once

#include "motionInitializer.hpp"

class SoemInitializer : public IMotionInitializer {
 public:
  bool motor_initial_connect(const char* ifname, int motor_count, MotorModel model, MotorModes mode) override;
  bool run_async() override;
  void stop() override;
  void close() override;

 private:
  bool opened_ = false;
  bool running_ = false;
  bool initialized_ = false;
};
