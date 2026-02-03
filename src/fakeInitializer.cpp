#include "fakeInitializer.hpp"
#include <iostream>
#include <chrono>

bool FakeInitializer::motor_initial_connect(const char* ifname, int motor_count, int cyclePeriod,MotorModes mode)
{
  (void)ifname;
  (void)motor_count;
  (void)mode;
  set_cycle_period_ms(cyclePeriod);
  opened_ = true;
  return true;
}

bool FakeInitializer::start_async(CyclicSession& session, AllMotors& motors, bool call_session)
{
  if (!opened_) return false;
  if (running_) return true;
  if (worker_.joinable()) {
    worker_.join();
  }
  set_call_session_enabled(call_session);
  reset_shutdown_notification();
  running_ = true;
  worker_ = std::thread(&FakeInitializer::run_loop_impl, this, std::ref(session), std::ref(motors));
  return true;
}

bool FakeInitializer::initializer_run_async(CyclicSession& session, AllMotors& motors)
{
  return start_async(session, motors, true);
}

bool FakeInitializer::initializer_run_async_io_only(CyclicSession& session, AllMotors& motors)
{
  return start_async(session, motors, false);
}

void FakeInitializer::run_loop_impl(CyclicSession& session, AllMotors& motors)
{
  using clock = std::chrono::steady_clock;
  auto next = clock::now();
  auto last = next;
  long long dt_sum_ns = 0;
  long long dt_min_ns = 0;
  long long dt_max_ns = 0;
  int dt_samples = 0;
  const int dt_log_interval = 500;
  while (running_) {
    next += std::chrono::milliseconds(cycle_period_ms());

    auto now = clock::now();
    long long dt_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(now - last).count();
    last = now;
    if (dt_samples == 0) {
      dt_min_ns = dt_ns;
      dt_max_ns = dt_ns;
    } else {
      if (dt_ns < dt_min_ns) dt_min_ns = dt_ns;
      if (dt_ns > dt_max_ns) dt_max_ns = dt_ns;
    }
    dt_sum_ns += dt_ns;
    ++dt_samples;
    if (dt_samples >= dt_log_interval) {
      log_cycle_stats(dt_sum_ns, dt_min_ns, dt_max_ns, dt_samples);
      dt_sum_ns = 0;
      dt_min_ns = 0;
      dt_max_ns = 0;
      dt_samples = 0;
    }

    if (get_call_session_enabled()) {
      bool cycle_shutdown_request = false;
      session.run(motors, cycle_shutdown_request);
      if (cycle_shutdown_request) {
        session.setCallback(nullptr);
        notify_shutdown_requested();
      }
    }

    std::this_thread::sleep_until(next);
  }
  printf("FakeInitializer loop thread exit.\n");
}
bool FakeInitializer::get_slave_count(const char* ifname, int& count)
{
  (void)ifname;
  count = 0;
  return false;
}

std::thread& FakeInitializer::worker_thread() { return worker_; }

std::atomic<bool>& FakeInitializer::running_flag() { return running_; }

void FakeInitializer::motor_stop()
{
  running_ = false;
  if (worker_.joinable()) {
    worker_.join();
  }
}

void FakeInitializer::motor_close()
{
  motor_stop();
  opened_ = false;
}
