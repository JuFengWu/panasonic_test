#include "motionSystem.hpp"
#include "fakeInitializer.hpp"
#include "fakeMotor.hpp"
#include "panasonicEthercatInitializer.hpp"

#include <stdexcept>

void AllMotors::initialize(MotorModel model, MotorModes mode, int count) {
  model_ = model;
  mode_ = mode;
  motors_.clear();
  motors_.reserve(static_cast<size_t>(count));
  for (int i = 1; i <= count; ++i) {
    motors_.push_back(create_motor(i));
  }
}

Motor& AllMotors::motor(int id) {
  if (motors_.empty()) {
    throw std::runtime_error("AllMotors not initialized");
  }
  if (id <= 0) {
    throw std::out_of_range("Motor id must be >= 1");
  }
  size_t idx = static_cast<size_t>(id - 1);
  if (idx >= motors_.size()) {
    throw std::out_of_range("Motor id exceeds initialized count");
  }
  return *motors_.at(idx);
}

int AllMotors::count() const { return static_cast<int>(motors_.size()); }

std::unique_ptr<Motor> AllMotors::create_motor(int id) {
  switch (model_) {
    case PanasonicA6MotorType:
      return std::make_unique<PanasonicA6>(id, mode_);
    default:
      return std::make_unique<FakeMotor>(id, mode_);
  }
}

MotionSystem::MotionSystem(MotorModel model, const char* ifname)
    : model_(model), ifname_(ifname ? ifname : "") {
  initializer_ = create_initializer(model_);
}

bool MotionSystem::start_connect(int motor_count, MotorModes mode) {
  motor_count_ = motor_count;
  mode_ = mode;
  motors_.initialize(model_, mode_, motor_count_);
  initializer_ = create_initializer(model_);
  if (!initializer_) {
    return false;
  }
  auto fatal_handler = [this]() {
    if (!initializer_) {
      return;
    }
    bool expected = false;
    if (closing_.compare_exchange_strong(expected, true)) {
      initializer_->motor_close();
    }
  };
  for (int i = 1; i <= motor_count_; ++i) {
    motors_.motor(i).set_fatal_handler(fatal_handler);
  }
  if (ifname_.empty()) {
    return false;
  }
  bool isInitialOK = initializer_->motor_initial_connect(ifname_.c_str(), motor_count, mode);

  for (int i = 1; i <= motor_count_; ++i) {
    motors_.motor(i).set_mode(mode);
  }
  return isInitialOK;
}

AllMotors& MotionSystem::motors() { return motors_; }

CyclicSession& MotionSystem::session() { return session_; }

bool MotionSystem::run_async() {
  if (!initializer_) {
    return false;
  }
  return initializer_->run_async(session_, motors_);
}

bool MotionSystem::get_slave_count(int& count) {
  if (!initializer_) {
    count = 0;
    return false;
  }
  if (ifname_.empty()) {
    count = 0;
    return false;
  }
  return initializer_->get_slave_count(ifname_.c_str(), count);
}

bool MotionSystem::drive_motors() {
  if (!initializer_) {
    return false;
  }
  return initializer_->initial_drive_motors();
}

void MotionSystem::set_cycle_log_enabled(bool enabled) {
  if (!initializer_) {
    return;
  }
  initializer_->set_cycle_log_enabled(enabled);
}

void MotionSystem::stop() {
  if (initializer_) {
    initializer_->motor_stop();
  }
}

void MotionSystem::close() {
  if (initializer_) {
    initializer_->motor_close();
  }
}

std::unique_ptr<IMotionInitializer> MotionSystem::create_initializer(MotorModel model) {
  switch (model) {
    case FakeMotorType:
      return std::make_unique<FakeInitializer>();
    case PanasonicA6MotorType:
      return std::make_unique<PanasonicEthercatInitializer>();
    default:
      return nullptr;
  }
}


