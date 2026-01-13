#include "motionInitializer.hpp"
#include <iostream>
#include "motionSystem.hpp"

void IMotionInitializer::handle_shutdown_request(AllMotors& motors,
                                                 bool shutdown_requested,
                                                 std::atomic<bool>& servo_off_inflight,
                                                 std::atomic<bool>& servo_off_done,
                                                 bool& shutdown_countdown_started,
                                                 int& shutdown_cycles_left,
                                                 int shutdown_hold_cycles,
                                                 std::thread& servo_off_thread)
{
  if (!shutdown_requested) {
    return;
  }

  if (!servo_off_done.load() && !servo_off_inflight.load()) {
    servo_off_inflight.store(true);
    servo_off_thread = std::thread([&motors, &servo_off_inflight, &servo_off_done]() {
      for (int i = 1; i <= motors.count(); ++i) {
        motors.motor(i).servo_off();
      }
      servo_off_done.store(true);
      servo_off_inflight.store(false);
    });
  }

  if (servo_off_done.load() && !shutdown_countdown_started) {
    shutdown_cycles_left = shutdown_hold_cycles;
    shutdown_countdown_started = true;
    std::cout<<"servo off finish"<<std::endl;
  } else if (shutdown_countdown_started && shutdown_cycles_left > 0) {
    --shutdown_cycles_left;
    if (shutdown_cycles_left == 0) {
      running_flag().store(false);
    }
  }
}

bool IMotionInitializer::initial_drive_motors()
{
  if (worker_thread().joinable()) {
    worker_thread().join();
  }
  running_flag().store(false);
  return true;
}
