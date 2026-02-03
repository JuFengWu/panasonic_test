#include "motionInitializer.hpp"
#include <chrono>
#include <cstdio>
#include <ctime>
#include <filesystem>

bool IMotionInitializer::initial_drive_motors()
{
  std::unique_lock<std::mutex> lock(shutdown_mutex_);
  shutdown_cv_.wait(lock, [this]() { return shutdown_notified_; });
  return true;
}

void IMotionInitializer::set_cycle_log_enabled(bool enabled)
{
  log_enabled_.store(enabled);
}

void IMotionInitializer::set_cycle_period_ms(int ms)
{
  if (ms <= 0) {
    ms = 1;
  }
  cycle_period_ms_.store(ms);
}

int IMotionInitializer::cycle_period_ms() const
{
  return cycle_period_ms_.load();
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


void IMotionInitializer::log_cycle_stats(long long dt_sum_ns, long long dt_min_ns, long long dt_max_ns, int dt_samples)
{
  if (!log_enabled_.load()) {
    return;
  }
  if (dt_samples <= 0) {
    return;
  }
  std::lock_guard<std::mutex> lock(log_mutex_);
  if (!log_file_.is_open()) {
    std::filesystem::create_directories("logs");
    log_file_.open("logs/cycle.log", std::ios::app);
  }
  if (!log_file_.is_open()) {
    return;
  }
  using clock = std::chrono::system_clock;
  auto now = clock::now();
  auto secs = std::chrono::time_point_cast<std::chrono::seconds>(now);
  auto ns = std::chrono::duration_cast<std::chrono::nanoseconds>(now - secs).count();
  std::time_t tt = clock::to_time_t(secs);
  std::tm tm{};
#if defined(_WIN32)
  localtime_s(&tm, &tt);
#else
  localtime_r(&tt, &tm);
#endif
  char buf[64];
  std::snprintf(buf, sizeof(buf), "%04d-%02d-%02d %02d:%02d:%02d",
                tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday,
                tm.tm_hour, tm.tm_min, tm.tm_sec);
  double avg_us = static_cast<double>(dt_sum_ns) / dt_samples / 1000.0;
  double min_us = static_cast<double>(dt_min_ns) / 1000.0;
  double max_us = static_cast<double>(dt_max_ns) / 1000.0;
  log_file_ << buf << "." << ns << " avg_us=" << avg_us
            << " min_us=" << min_us << " max_us=" << max_us << '\n';
  log_file_.flush();
}
