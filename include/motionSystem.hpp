#pragma once

#include "cyclicSession.hpp"
#include "motionInitializer.hpp"
#include "panasonicA6B.hpp"

#include <memory>
#include <atomic>
#include <vector>

class AllMotors {
 public:
  void initialize(MotorModel model, MotorModes mode, int count);
  Motor& motor(int id);
  int count() const;

 private:
  std::unique_ptr<Motor> create_motor(int id);

  std::vector<std::unique_ptr<Motor>> motors_;
  MotorModel model_ = PanasonicA6BMotorType;
  MotorModes mode_ = PP_Mode;
};

class MotionSystem {
 public:
  MotionSystem() = default;

  bool start_connect(const char* ifname, int motor_count, MotorModel model, MotorModes mode=PP_Mode);

  AllMotors& motors();
  CyclicSession& session();
  bool run_async();
  void stop();
  void close();

 private:
  std::unique_ptr<IMotionInitializer> create_initializer(MotorModel model);

  AllMotors motors_;
  CyclicSession session_;
  MotorModel model_ = PanasonicA6BMotorType;
  MotorModes mode_ = PP_Mode;
  int motor_count_ = 0;
  std::unique_ptr<IMotionInitializer> initializer_;
  std::atomic<bool> closing_{false};
};
