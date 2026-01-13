#include "fakeInitializer.hpp"
#include <iostream>
#include <chrono>

bool FakeInitializer::motor_initial_connect(const char* ifname, int motor_count, MotorModes mode)
{
  (void)ifname;
  (void)motor_count;
  (void)mode;
  opened_ = true;
  return true;
}

bool FakeInitializer::run_async(CyclicSession& session, AllMotors& motors)
{
  if (!opened_) return false;
  if (running_) return true;
  if (worker_.joinable()) {
    worker_.join();
  }
  reset_shutdown_notification();
  running_ = true;
  worker_ = std::thread([this, &session, &motors]() {
    using clock = std::chrono::steady_clock;
    auto next = clock::now();
    while (running_) {
      next += std::chrono::milliseconds(4);

      bool cycle_shutdown_request = false;
      session.run(motors, cycle_shutdown_request);
      if (cycle_shutdown_request) {
        session.setCallback(nullptr);
        notify_shutdown_requested();
      }
      std::this_thread::sleep_until(next);
    }
    printf("FakeInitializer loop thread exit.\n");
  });
  return true;
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
