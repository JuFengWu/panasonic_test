#include "fakeInitializer.hpp"

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
  running_ = true;
  worker_ = std::thread([this, &session, &motors]() {
    using clock = std::chrono::steady_clock;
    auto next = clock::now();
    while (running_) {
      next += std::chrono::milliseconds(4);
      bool break_loop = false;
      session.run(motors, break_loop);
      if (break_loop) {
        running_ = false;
        break;
      }
      std::this_thread::sleep_until(next);
    }
  });
  return true;
}

void FakeInitializer::stop()
{
  running_ = false;
  if (worker_.joinable()) {
    worker_.join();
  }
}

void FakeInitializer::close()
{
  stop();
  opened_ = false;
}
