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
  bool motor_initial_connect(const char* ifname, int motor_count, int cyclePeriod,MotorModes mode) override;
  bool run_async(CyclicSession& session, AllMotors& motors) override;
  bool run_async_io(CyclicSession& session, AllMotors& motors) override;
  bool get_slave_count(const char* ifname, int& count) override;
  void motor_stop() override;
  void motor_close() override;

 private:
  bool start_async(CyclicSession& session, AllMotors& motors, bool call_session);
  void run_loop_impl(CyclicSession& session, AllMotors& motors);

  bool opened_ = false;
  std::atomic<bool> running_{false};
  bool initialized_ = false;
  char ioMap[4096];
  std::thread worker_;
  bool setup_minasa6b_pdo_mapping4(uint16 slave);
  bool setInterpolationTimePeriod(uint16 slave, int us);
  void print_state();
  bool set_profile_motion_params(uint16 slave);
  void init_motion_params_pdo(uint16 slave, MotorModes mode);
  bool shutdown_ecat();
  AllMotors* motors_ = nullptr;

 protected:
  std::thread& worker_thread() override;
  std::atomic<bool>& running_flag() override;
};
