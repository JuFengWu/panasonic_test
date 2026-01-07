#pragma once

#include "motionInitializer.hpp"
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include "utilities.hpp"
#include <unistd.h>
class SoemInitializer : public IMotionInitializer {
 public:
  bool motor_initial_connect(const char* ifname, int motor_count, MotorModes mode) override;
  bool run_async() override;
  void stop() override;
  void close() override;

 private:
  bool opened_ = false;
  bool running_ = false;
  bool initialized_ = false;
  char ioMap[4096];
  bool setup_minasa6b_pdo_mapping4(uint16 slave);
  void print_state();
  bool set_profile_motion_params(uint16 slave);
};
