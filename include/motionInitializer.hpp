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
  virtual bool drive_motors() {
    if (worker_thread().joinable()) {
      worker_thread().join();
    }
    running_flag().store(false);
    return true;
  }
  virtual void motor_stop() = 0;
  virtual void motor_close() = 0;

 protected:
  virtual std::thread& worker_thread() = 0;
  virtual std::atomic<bool>& running_flag() = 0;
};
