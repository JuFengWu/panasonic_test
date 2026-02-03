#pragma once

#include "cyclicSession.hpp"
#include "motionInitializer.hpp"
#include "panasonicA6.hpp"

#include <memory>
#include <atomic>
#include <string>
#include <vector>

class AllMotors {
 public:
  void initialize(MotorModel model, MotorModes mode, int count);
  Motor& motor(int id);
  int count() const;

 private:
  std::unique_ptr<Motor> create_motor(int id);

  std::vector<std::unique_ptr<Motor>> motors_;
  MotorModel model_ = PanasonicA6MotorType;
  MotorModes mode_ = PP_Mode;
};

class MotionSystem {
 public:
  MotionSystem() = default;
  MotionSystem(MotorModel model, const char* ifname);

  bool start_connect(int motor_count, int cyclePeriod,MotorModes mode=PP_Mode);

  AllMotors& motors();
  CyclicSession& session();
  bool run_async();
  bool get_slave_count(int& count);
  bool drive_motors();
  void set_cycle_log_enabled(bool enabled);
  void stop();
  void close();

 private:
  std::unique_ptr<IMotionInitializer> create_initializer(MotorModel model);

  AllMotors motors_;
  CyclicSession session_;
  MotorModel model_ = PanasonicA6MotorType;
  MotorModes mode_ = PP_Mode;
  int motor_count_ = 0;
  std::string ifname_;
  std::unique_ptr<IMotionInitializer> initializer_;
  std::atomic<bool> closing_{false};
};


