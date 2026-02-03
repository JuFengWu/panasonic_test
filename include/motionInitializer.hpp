#pragma once

#include "cyclicSession.hpp"
#include "motors.hpp"

#include <atomic>
#include <condition_variable>
#include <fstream>
#include <mutex>
#include <thread>

class AllMotors;

enum class MotionInitError {
  Ok,
  AlreadyRunning,
  NotRunning,
  StateMachineFault,
  DataExchangeTimeout,
  ServoOffFailed,
  Unknown
};

class IMotionInitializer {
 public:
  virtual ~IMotionInitializer() = default;

  virtual bool motor_initial_connect(const char* ifname, int motor_count, int cyclePeriod,MotorModes mode) = 0;
  virtual bool run_async(CyclicSession& session, AllMotors& motors) = 0;
  virtual bool get_slave_count(const char* ifname, int& count) = 0;
  bool initial_drive_motors();
  void set_cycle_log_enabled(bool enabled);
  void log_cycle_stats(long long dt_sum_ns, long long dt_min_ns, long long dt_max_ns, int dt_samples);
 virtual void motor_stop() = 0;
 virtual void motor_close() = 0;

 protected:
  virtual std::thread& worker_thread() = 0;
  virtual std::atomic<bool>& running_flag() = 0;
  void set_cycle_period_ms(int ms);
  int cycle_period_ms() const;
  void reset_shutdown_notification();
  void notify_shutdown_requested();

 private:
  std::mutex log_mutex_;
  std::ofstream log_file_;
  std::atomic<bool> log_enabled_{true};
  std::atomic<int> cycle_period_ms_{4};
  std::mutex shutdown_mutex_;
  std::condition_variable shutdown_cv_;
  bool shutdown_notified_ = false;
};
