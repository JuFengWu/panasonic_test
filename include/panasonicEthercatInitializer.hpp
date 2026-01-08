#pragma once

#include "motionInitializer.hpp"
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <unistd.h>
#include <atomic>
#include <thread>

#include "utilities.hpp"

class PanasonicEthercatInitializer : public IMotionInitializer {
 public:
  bool motor_initial_connect(const char* ifname, int motor_count, MotorModes mode) override;
  bool run_async(CyclicSession& session, AllMotors& motors) override;
  void stop() override;
  void close() override;

 private:
  void run_loop(CyclicSession& session, AllMotors& motors);

  bool opened_ = false;
  std::atomic<bool> running_{false};
  bool initialized_ = false;
  char ioMap[4096];
  std::thread worker_;
  bool setup_minasa6b_pdo_mapping4(uint16 slave);
  void print_state();
  bool set_profile_motion_params(uint16 slave);
};
