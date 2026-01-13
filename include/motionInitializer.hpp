#pragma once

#include "cyclicSession.hpp"
#include "motors.hpp"

#include <atomic>
#include <thread>

class AllMotors;

class IMotionInitializer {
 public:
  virtual ~IMotionInitializer() = default;

  virtual bool motor_initial_connect(const char* ifname, int motor_count, MotorModes mode) = 0;
  virtual bool run_async(CyclicSession& session, AllMotors& motors) = 0;
  bool initial_drive_motors();
  virtual void motor_stop() = 0;
  virtual void motor_close() = 0;

 protected:
  virtual std::thread& worker_thread() = 0;
  virtual std::atomic<bool>& running_flag() = 0;
  void handle_shutdown_request(AllMotors& motors,
                               bool shutdown_requested,
                               std::atomic<bool>& servo_off_inflight,
                               std::atomic<bool>& servo_off_done,
                               bool& shutdown_countdown_started,
                               int& shutdown_cycles_left,
                               int shutdown_hold_cycles,
                               std::thread& servo_off_thread);
};
