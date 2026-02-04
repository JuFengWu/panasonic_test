#include "motionSystem.hpp"

#include <cstdio>
#include <iostream>
#include <thread>

static void read_position_callback(AllMotors& motors, bool& cycle_shutdown_request)
{
  int pos = motors.motor(1).get_current_position();
  std::cout << "pos is " << pos << std::endl;
}

int main()
{
  // Beginner Demo
  // Goal: connect -> start loop -> servo on -> send simple commands -> servo off -> stop

  // Step 0: Basic configuration (edit these for your setup)
  const char* ifname = "eno1";    // Network interface name
  const int motor_count = 1;      // Number of motors
  const int cycle_period_ms = 1;  // Control cycle in milliseconds
  MotorModes mode = PP_Mode;      // Choose one mode for a simple demo

  // Step 1: Create system and connect
  MotionSystem sys(PanasonicA6MotorType, ifname);
  if (!sys.start_connect(motor_count, cycle_period_ms, mode)) {
    printf("start_connect failed\n");
    return 1;
  }
  printf("connected\n");

  // Step 2: Start background loop (control loop enabled)
  if (!sys.run_async()) {
    printf("run_async failed\n");
    return 1;
  }
  printf("loop started\n");

  // Step 3: Use callback to read current position (no control in callback)
  sys.session().setCallback(read_position_callback);

  // Step 4: Servo ON (enable motor)
  sys.motors().motor(1).servo_on();

  // Step 5: Set target position outside the loop thread
  // The async loop keeps communication alive; we update targets as needed here.
  sys.motors().motor(1).set_target_position(0);
  std::this_thread::sleep_for(std::chrono::seconds(2));
  sys.motors().motor(1).set_target_position(1000);

  // Step 6: Let it run for a short while
  std::this_thread::sleep_for(std::chrono::seconds(2));

  // Step 7: Servo OFF and close
  sys.motors().motor(1).servo_off();
  std::cout << "Closing motion system..." << std::endl;
  sys.close();
  printf("done\n");

  return 0;
}
