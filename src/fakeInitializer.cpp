#include "fakeInitializer.hpp"
#include "motionSystem.hpp"

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
    bool shutdown_requested = false;
    std::atomic<bool> servo_off_inflight{false};
    std::atomic<bool> servo_off_done{false};
    bool shutdown_countdown_started = false;
    int shutdown_cycles_left = 0;
    const int shutdown_hold_cycles = 50;
    std::thread servo_off_thread;
    while (running_) {
      next += std::chrono::milliseconds(4);

      session.run(motors, shutdown_requested);

      handle_shutdown_request(motors,
                              shutdown_requested,
                              servo_off_inflight,
                              servo_off_done,
                              shutdown_countdown_started,
                              shutdown_cycles_left,
                              shutdown_hold_cycles,
                              servo_off_thread);

      if (!running_) {
        break;
      }
      std::this_thread::sleep_until(next);
    }
    if (servo_off_thread.joinable()) {
      servo_off_thread.join();
    }
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
