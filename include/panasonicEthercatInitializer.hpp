#pragma once

#include "motionInitializer.hpp"
#include "Ethercat.hpp"
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <unistd.h>
#include <atomic>
#include <memory>
#include <thread>

class PanasonicEthercatInitializer : public IMotionInitializer {
 public:
  PanasonicEthercatInitializer();

  bool motor_initial_connect(const char* ifname, int motor_count, int cyclePeriod,MotorModes mode) override;
  bool initializer_run_async(CyclicSession& session, AllMotors& motors) override;
  bool initializer_run_async_io_only(CyclicSession& session, AllMotors& motors) override;
  bool get_slave_count(const char* ifname, int& count) override;
  std::shared_ptr<MyEthercat> get_ethercat() const override { return ethercat_; }
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
  bool setup_minasa6b_pdo_mapping4(std::uint16_t slave);
  bool setInterpolationTimePeriod(std::uint16_t slave, int us);
  void print_state();
  bool set_profile_motion_params(std::uint16_t slave);
  void init_motion_params_pdo(std::uint16_t slave, MotorModes mode);
  bool shutdown_ecat();
  AllMotors* motors_ = nullptr;
  std::shared_ptr<MyEthercat> ethercat_;

 protected:
  std::thread& worker_thread() override;
  std::atomic<bool>& running_flag() override;
};
