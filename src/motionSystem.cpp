#include "motionSystem.hpp"
#include "fakeInitializer.hpp"
#include "fakeMotor.hpp"
#include "panasonicEthercatInitializer.hpp"

void AllMotors::initialize(MotorModel model, MotorModes mode, int count)
{
  model_ = model;
  mode_ = mode;
  motors_.clear();
  motors_.reserve(static_cast<size_t>(count));
  for (int i = 1; i <= count; ++i) {
    motors_.push_back(create_motor(i));
  }
}

Motor& AllMotors::motor(int id)
{
  if (id <= 0) {
    id = 1;
  }
  size_t idx = static_cast<size_t>(id - 1);
  if (idx >= motors_.size()) {
    motors_.reserve(idx + 1);
    for (int i = static_cast<int>(motors_.size()) + 1; i <= id; ++i) {
      motors_.push_back(create_motor(i));
    }
  }
  return *motors_.at(idx);
}

int AllMotors::count() const { return static_cast<int>(motors_.size()); }

std::unique_ptr<Motor> AllMotors::create_motor(int id)
{
  switch (model_) {
    case PanasonicA6BMotorType:
      return std::make_unique<PanasonicA6B>(id, mode_);
    default:
      return std::make_unique<FakeMotor>(id, mode_);
  }
}

bool MotionSystem::start_connect(const char* ifname, int motor_count, MotorModel model, MotorModes mode)
{
  model_ = model;
  motor_count_ = motor_count;
  mode_ = mode;
  motors_.initialize(model_, mode_, motor_count_);
  initializer_ = create_initializer(model_);
  if (!initializer_) {
    return false;
  }
  return initializer_->motor_initial_connect(ifname, motor_count, mode);
}

AllMotors& MotionSystem::motors() { return motors_; }

CyclicSession& MotionSystem::session() { return session_; }

bool MotionSystem::run_async()
{
  if (!initializer_) return false;
  return initializer_->run_async();
}

void MotionSystem::stop()
{
  if (initializer_) {
    initializer_->stop();
  }
}

void MotionSystem::close()
{
  if (initializer_) {
    initializer_->close();
  }
}

std::unique_ptr<IMotionInitializer> MotionSystem::create_initializer(MotorModel model)
{
  switch (model) {
    case FakeMotorType:
      return std::make_unique<FakeInitializer>();
    case PanasonicA6BMotorType:
      return std::make_unique<PanasonicEthercatInitializer>();
    default:
      return nullptr;
  }
}
