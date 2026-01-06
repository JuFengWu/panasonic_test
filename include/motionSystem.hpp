#pragma once

#include "cyclicSession.hpp"
#include "panasonicA6B.hpp"

#include <memory>
#include <vector>

class Motors {
 public:
  void initialize(MotorModel model, MotorModes mode, int count);
  Motor& motor(int id);
  int count() const;

 private:
  std::unique_ptr<Motor> create_motor(int id);

  std::vector<std::unique_ptr<Motor>> motors_;
  MotorModel model_ = PanasonicA6BMotor;
  MotorModes mode_ = PP_Mode;
};

class MotionSystem {
 public:
  MotionSystem() = default;

  bool open(const char* ifname, int motor_count, MotorModel model, MotorModes mode=PP_Mode);
  Motors& motors();
  CyclicSession& session();
  bool run_async();
  void stop();
  void close();

 private:
  Motors motors_;
  CyclicSession session_;
  bool opened_ = false;
  bool running_ = false;
  bool initialized_ = false;
  MotorModel model_ = PanasonicA6BMotor;
  MotorModes mode_ = PP_Mode;
  int motor_count_ = 0;
};
