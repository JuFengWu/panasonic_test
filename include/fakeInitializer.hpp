#pragma once

#include "motionInitializer.hpp"

#include <atomic>
#include <thread>

class FakeInitializer : public IMotionInitializer {
 public:
  bool motor_initial_connect(const char* ifname, int motor_count,int cyclePeriod, MotorModes mode) override;
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
  std::thread worker_;

 protected:
  std::thread& worker_thread() override;
  std::atomic<bool>& running_flag() override;
};
