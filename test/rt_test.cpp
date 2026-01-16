#include <algorithm>
#include <atomic>
#include <chrono>
#include <climits>
#include <csignal>
#include <cstdlib>
#include <iostream>
#include <string>
#include <thread>
#include <vector>

static std::atomic<bool> g_running{true};

static void on_sigint(int) { g_running.store(false); }

struct Stats {
  long long min_ns = LLONG_MAX;
  long long max_ns = 0;
  long double sum_ns = 0;
  long long count = 0;
};

static void add_sample(Stats& s, long long v) {
  s.min_ns = std::min(s.min_ns, v);
  s.max_ns = std::max(s.max_ns, v);
  s.sum_ns += static_cast<long double>(v);
  s.count++;
}

static void print_help(const char* argv0) {
  std::cout
    << "Usage: " << argv0 << " [options]\n"
    << "Options:\n"
    << "  --duration-s N        Duration in seconds (default 10)\n"
    << "  --print-every N       Print running stats every N cycles (default 0 = end only)\n"
    << "  --expected-us N       Expected period in microseconds (default 4000)\n"
    << "\nNotes:\n"
    << "  Loop uses a fixed sleep_until period like FakeInitializer::run_async.\n";
}

static void print_summary(const Stats& s, const std::vector<long long>& samples, long long expected_ns) {
  if (s.count == 0) {
    std::cout << "No samples collected.\n";
    return;
  }
  auto sorted = samples;
  std::sort(sorted.begin(), sorted.end());

  auto pct = [&](double p) -> long long {
    if (sorted.empty()) return 0;
    double idx = p * (sorted.size() - 1);
    size_t i = static_cast<size_t>(idx);
    return sorted[i];
  };

  long double avg_ns = s.sum_ns / static_cast<long double>(s.count);
  long double expected_us = static_cast<long double>(expected_ns) / 1000.0L;

  std::cout << "Samples: " << s.count << "\n";
  std::cout << "Expected period: " << static_cast<double>(expected_us) << " us\n";
  std::cout << "Period (us): avg=" << static_cast<double>(avg_ns / 1000.0L)
            << " min=" << static_cast<double>(s.min_ns) / 1000.0
            << " max=" << static_cast<double>(s.max_ns) / 1000.0
            << "\n";
  std::cout << "Percentiles (us): "
            << "p50=" << static_cast<double>(pct(0.50)) / 1000.0
            << " p90=" << static_cast<double>(pct(0.90)) / 1000.0
            << " p99=" << static_cast<double>(pct(0.99)) / 1000.0
            << " p99.9=" << static_cast<double>(pct(0.999)) / 1000.0
            << "\n";
}

int main(int argc, char** argv) {
  int duration_s = 10;
  long long print_every = 0;
  long long expected_us = 4000;

  for (int i = 1; i < argc; i++) {
    std::string a = argv[i];
    auto need = [&](const char* name) -> const char* {
      if (i + 1 >= argc) {
        std::cerr << "Missing value for " << name << "\n";
        std::exit(2);
      }
      return argv[++i];
    };

    if (a == "--help" || a == "-h") {
      print_help(argv[0]);
      return 0;
    } else if (a == "--duration-s") {
      duration_s = std::atoi(need("--duration-s"));
    } else if (a == "--print-every") {
      print_every = std::atoll(need("--print-every"));
    } else if (a == "--expected-us") {
      expected_us = std::atoll(need("--expected-us"));
    } else {
      std::cerr << "Unknown arg: " << a << "\n";
      print_help(argv[0]);
      return 2;
    }
  }

  std::signal(SIGINT, on_sigint);

  Stats stats{};
  std::vector<long long> samples;
  samples.reserve(static_cast<size_t>(duration_s) * 300);
  std::atomic<bool> running{true};

  std::thread worker([&]() {
    using clock = std::chrono::steady_clock;
    auto next = clock::now();
    auto last = next;

    while (running.load()) {
      next += std::chrono::microseconds(expected_us);

      auto now = clock::now();
      long long dt_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(now - last).count();
      last = now;
      add_sample(stats, dt_ns);
      samples.push_back(dt_ns);

      if (print_every > 0 && (stats.count % print_every) == 0) {
        long double avg_ns = stats.sum_ns / static_cast<long double>(stats.count);
        std::cout << "cycles=" << stats.count
                  << " avg_us=" << static_cast<double>(avg_ns / 1000.0L)
                  << " min_us=" << static_cast<double>(stats.min_ns) / 1000.0
                  << " max_us=" << static_cast<double>(stats.max_ns) / 1000.0
                  << "\n";
      }

      std::this_thread::sleep_until(next);
    }
  });

  auto end_time = std::chrono::steady_clock::now() + std::chrono::seconds(duration_s);
  while (g_running.load() && std::chrono::steady_clock::now() < end_time) {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  running.store(false);
  if (worker.joinable()) {
    worker.join();
  }

  std::cout << "\n=== Summary ===\n";
  print_summary(stats, samples, expected_us * 1000LL);
  return 0;
}
