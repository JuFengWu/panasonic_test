#pragma once

#include "cyclicSession.hpp"
#include "panasonicA6B.hpp"

#include <memory>
#include <vector>

class Motors {
 public:
  void initialize(MotorModel model, MotorModes mode, int count)
  {
    model_ = model;
    mode_ = mode;
    motors_.clear();
    motors_.reserve(static_cast<size_t>(count));
    for (int i = 1; i <= count; ++i) {
      motors_.push_back(create_motor(i));
    }
  }

  Motor& motor(int id)
  {
    if (id <= 0) {
      id = 1;
    }
    int idx = id - 1;
    
    return *motors_.at(idx);
  }

  int count() const { return motors_.size(); }

 private:
  std::unique_ptr<Motor> create_motor(int id)
  {
    switch (model_) {
      case PanasonicA6BMotor:
        return std::make_unique<PanasonicA6B>(id, mode_);
      default:
        return std::make_unique<PanasonicA6B>(id, mode_);
    }
  }

  std::vector<std::unique_ptr<Motor>> motors_;
  MotorModel model_ = PanasonicA6BMotor;
  MotorModes mode_ = PP_Mode;
};

class MotionSystem {
 public:
  MotionSystem() = default;

  bool open(const char* ifname, int motor_count, MotorModel model, MotorModes mode=PP_Mode)
  {
    (void)ifname;
    opened_ = true;
    model_ = model;
    motor_count_ = motor_count;
    mode_ = mode;
    motors_.initialize(model_, mode_, motor_count_);
    initialized_ = true;
    return true;
  }

  Motors& motors()
  {
    return motors_;
  }

  CyclicSession& session() { return session_; }

  bool run_async()
  {
    if (!opened_) return false;
    running_ = true;
    return true;
  }

  void stop() { running_ = false; }

  void close()
  {
    running_ = false;
    opened_ = false;
  }

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
