#include "motionInitializer.hpp"

bool IMotionInitializer::initial_drive_motors()
{
  std::unique_lock<std::mutex> lock(shutdown_mutex_);
  shutdown_cv_.wait(lock, [this]() { return shutdown_notified_; });
  return true;
}

void IMotionInitializer::reset_shutdown_notification()
{
  std::lock_guard<std::mutex> lock(shutdown_mutex_);
  shutdown_notified_ = false;
}

void IMotionInitializer::notify_shutdown_requested()
{
  std::lock_guard<std::mutex> lock(shutdown_mutex_);
  shutdown_notified_ = true;
  shutdown_cv_.notify_all();
}
