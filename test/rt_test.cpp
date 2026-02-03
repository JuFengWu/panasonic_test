#include <algorithm>
#include <atomic>
#include <chrono>
#include <climits>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <string>
#include <thread>
#include <vector>

#include <fcntl.h>
#include <sched.h>
#include <sys/resource.h>
#include <sys/mman.h>
#include <time.h>
#include <unistd.h>

static std::atomic<bool> g_running{true};

static void on_sigint(int) { g_running.store(false); }

static void burn_some_cpu(int burn_us) {
  if (burn_us <= 0) return;
  auto start = std::chrono::steady_clock::now();
  auto dur = std::chrono::microseconds(burn_us);
  while (std::chrono::steady_clock::now() - start < dur) {
    asm volatile("" ::: "memory");
  }
}

static bool set_rt_fifo(pthread_t thread, int prio) {
  sched_param sp{};
  sp.sched_priority = prio;
  if (pthread_setschedparam(thread, SCHED_FIFO, &sp) != 0) {
    std::perror("pthread_setschedparam(SCHED_FIFO) failed");
    return false;
  }
  return true;
}

static bool set_affinity(pthread_t thread, int cpu) {
  cpu_set_t set;
  CPU_ZERO(&set);
  CPU_SET(cpu, &set);
  if (pthread_setaffinity_np(thread, sizeof(set), &set) != 0) {
    std::perror("pthread_setaffinity_np failed");
    return false;
  }
  return true;
}

static bool lock_memory() {
  if (mlockall(MCL_CURRENT | MCL_FUTURE) != 0) {
    std::perror("mlockall failed");
    return false;
  }
  return true;
}

static void prefault_stack(size_t kb) {
  if (kb == 0) return;
  size_t bytes = kb * 1024;
  rlimit lim{};
  if (getrlimit(RLIMIT_STACK, &lim) == 0 && lim.rlim_cur != RLIM_INFINITY) {
    size_t max_bytes = static_cast<size_t>(lim.rlim_cur);
    if (max_bytes > 0 && bytes > max_bytes / 4) {
      bytes = max_bytes / 4;
    }
  }
  if (bytes == 0) return;
  volatile char* buf = (volatile char*)alloca(bytes);
  for (size_t i = 0; i < bytes; i += 4096) buf[i] = 0;
}

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
    << "  --rt                 Enable RT: SCHED_FIFO + mlockall + prefault stack\n"
    << "  --prio N              RT priority (default 80)\n"
    << "  --cpu N               Pin thread to CPU core N (default: no pin)\n"
    << "  --period-us N         Period in microseconds (default 4000)\n"
    << "  --duration-s N        Duration in seconds (default 10)\n"
    << "  --burn-us N           Busy-work each cycle to simulate load (default 0)\n"
    << "  --prefault-kb N       Prefault stack size in KB (default 256, 0 disables)\n"
    << "  --print-every N       Print running stats every N cycles (default 0 = end only)\n"
    << "\nNotes:\n"
    << "  Loop uses a fixed sleep_until period like FakeInitializer::initializer_run_async.\n";
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
  bool enable_rt = false;
  int prio = 80;
  int cpu = -1;
  int period_us = 1000;
  int duration_s = 10;
  int burn_us = 0;
  size_t prefault_kb = 256;
  long long print_every = 0;

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
    } else if (a == "--rt") {
      enable_rt = true;
    } else if (a == "--prio") {
      prio = std::atoi(need("--prio"));
    } else if (a == "--cpu") {
      cpu = std::atoi(need("--cpu"));
    } else if (a == "--period-us") {
      period_us = std::atoi(need("--period-us"));
    } else if (a == "--duration-s") {
      duration_s = std::atoi(need("--duration-s"));
    } else if (a == "--burn-us") {
      burn_us = std::atoi(need("--burn-us"));
    } else if (a == "--prefault-kb") {
      prefault_kb = static_cast<size_t>(std::atoll(need("--prefault-kb")));
    } else if (a == "--print-every") {
      print_every = std::atoll(need("--print-every"));
    } else {
      std::cerr << "Unknown arg: " << a << "\n";
      print_help(argv[0]);
      return 2;
    }
  }

  std::signal(SIGINT, on_sigint);

  if (enable_rt) {
    std::cout << "[RT] enabling SCHED_FIFO, mlockall, prefault...\n";
    if (!lock_memory()) {
      std::cerr << "Continuing without mlockall (this may increase jitter).\n";
    }
    prefault_stack(prefault_kb);
  }

  Stats stats{};
  std::vector<long long> samples;
  samples.reserve(static_cast<size_t>(duration_s) * 300);
  std::atomic<bool> running{true};

  std::thread worker([&]() {
    using clock = std::chrono::steady_clock;
    auto next = clock::now();
    long long period_ns = static_cast<long long>(period_us) * 1000LL;

    while (running.load()) {
      next += std::chrono::nanoseconds(period_ns);

      std::this_thread::sleep_until(next);

      auto now = clock::now();
      long long lateness_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(now - next).count();
      if (lateness_ns < 0) lateness_ns = 0;
      add_sample(stats, lateness_ns);
      samples.push_back(lateness_ns);

      if (print_every > 0 && (stats.count % print_every) == 0) {
        long double avg_ns = stats.sum_ns / static_cast<long double>(stats.count);
        std::cout << "cycles=" << stats.count
                  << " avg_us=" << static_cast<double>(avg_ns / 1000.0L)
                  << " min_us=" << static_cast<double>(stats.min_ns) / 1000.0
                  << " max_us=" << static_cast<double>(stats.max_ns) / 1000.0
                  << "\n";
      }

      burn_some_cpu(burn_us);

      auto now2 = clock::now();
      while (now2 > next) {
        next += std::chrono::nanoseconds(period_ns);
      }
    }
  });

  if (enable_rt || cpu >= 0) {
    if (cpu >= 0) {
      if (!set_affinity(worker.native_handle(), cpu)) {
        std::cerr << "Continuing without CPU pin.\n";
      }
    }
    if (enable_rt) {
      if (!set_rt_fifo(worker.native_handle(), prio)) {
        std::cerr << "Failed to enable SCHED_FIFO. Are you running as root (sudo) or have CAP_SYS_NICE?\n";
        std::cerr << "Continuing without RT scheduling.\n";
      }
    }
  }

  auto end_time = std::chrono::steady_clock::now() + std::chrono::seconds(duration_s);
  while (g_running.load() && std::chrono::steady_clock::now() < end_time) {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  running.store(false);
  if (worker.joinable()) {
    worker.join();
  }

  std::cout << "\n=== Summary ===\n";
  print_summary(stats, samples, static_cast<long long>(period_us) * 1000LL);
  return 0;
}



//./test_rt --rt --prefault-kb 0
