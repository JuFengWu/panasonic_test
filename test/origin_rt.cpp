#include <algorithm>
#include <atomic>
#include <cerrno>
#include <chrono>
#include <cinttypes>
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
#include <sys/mman.h>
#include <time.h>
#include <unistd.h>

static std::atomic<bool> g_running{true};

static void on_sigint(int) { g_running.store(false); }

static inline long long diff_ns(const timespec& a, const timespec& b) {
  // a - b
  return (a.tv_sec - b.tv_sec) * 1000000000LL + (a.tv_nsec - b.tv_nsec);
}

static inline timespec add_ns(timespec t, long long ns) {
  t.tv_nsec += ns;
  while (t.tv_nsec >= 1000000000L) {
    t.tv_nsec -= 1000000000L;
    t.tv_sec += 1;
  }
  while (t.tv_nsec < 0) {
    t.tv_nsec += 1000000000L;
    t.tv_sec -= 1;
  }
  return t;
}

static void burn_some_cpu(int burn_us) {
  // optional: simulate work (very light)
  if (burn_us <= 0) return;
  auto start = std::chrono::steady_clock::now();
  auto dur = std::chrono::microseconds(burn_us);
  while (std::chrono::steady_clock::now() - start < dur) {
    asm volatile("" ::: "memory");
  }
}

static bool set_rt_fifo(int prio) {
  sched_param sp{};
  sp.sched_priority = prio;
  if (pthread_setschedparam(pthread_self(), SCHED_FIFO, &sp) != 0) {
    std::perror("pthread_setschedparam(SCHED_FIFO) failed");
    return false;
  }
  return true;
}

static bool set_affinity(int cpu) {
  cpu_set_t set;
  CPU_ZERO(&set);
  CPU_SET(cpu, &set);
  if (pthread_setaffinity_np(pthread_self(), sizeof(set), &set) != 0) {
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

static void prefault_stack() {
  // Touch some stack pages to avoid page faults later
  constexpr size_t kSize = 8 * 1024 * 1024;
  volatile char* buf = (volatile char*)alloca(kSize);
  for (size_t i = 0; i < kSize; i += 4096) buf[i] = 0;
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
    << "  --raw-clock           Use CLOCK_MONOTONIC_RAW if available\n"
    << "  --print-every N       Print running stats every N cycles (default 0 = end only)\n"
    << "\nExamples:\n"
    << "  Non-RT: " << argv0 << " --duration-s 10 --period-us 4000\n"
    << "  RT:     sudo " << argv0 << " --rt --prio 80 --cpu 2 --duration-s 10 --period-us 4000\n";
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
  s.sum_ns += (long double)v;
  s.count++;
}

static void print_summary(const Stats& s, const std::vector<long long>& samples) {
  if (s.count == 0) return;
  auto sorted = samples;
  std::sort(sorted.begin(), sorted.end());

  auto pct = [&](double p) -> long long {
    if (sorted.empty()) return 0;
    double idx = p * (sorted.size() - 1);
    size_t i = (size_t)idx;
    return sorted[i];
  };

  long double avg = s.sum_ns / (long double)s.count;

  std::cout << "Samples: " << s.count << "\n";
  std::cout << "Lateness (wake - scheduled), microseconds:\n";
  std::cout << "  avg_us=" << (double)(avg / 1000.0L)
            << "  min_us=" << (double)s.min_ns / 1000.0
            << "  max_us=" << (double)s.max_ns / 1000.0 << "\n";
  std::cout << "Percentiles (us): "
            << "p50=" << (double)pct(0.50) / 1000.0
            << "  p90=" << (double)pct(0.90) / 1000.0
            << "  p99=" << (double)pct(0.99) / 1000.0
            << "  p99.9=" << (double)pct(0.999) / 1000.0
            << "\n";
}

int main(int argc, char** argv) {
  bool enable_rt = false;
  int prio = 80;
  int cpu = -1;
  int period_us = 4000;
  int duration_s = 10;
  int burn_us = 0;
  bool raw_clock = false;
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
    } else if (a == "--raw-clock") {
      raw_clock = true;
    } else if (a == "--print-every") {
      print_every = std::atoll(need("--print-every"));
    } else {
      std::cerr << "Unknown arg: " << a << "\n";
      print_help(argv[0]);
      return 2;
    }
  }

  std::signal(SIGINT, on_sigint);

  clockid_t clk_id = CLOCK_MONOTONIC;
#ifdef CLOCK_MONOTONIC_RAW
  if (raw_clock) clk_id = CLOCK_MONOTONIC_RAW;
#endif

  if (enable_rt) {
    std::cout << "[RT] enabling SCHED_FIFO, mlockall, prefault...\n";
    if (!lock_memory()) {
      std::cerr << "Continuing without mlockall (this may increase jitter).\n";
    }
    prefault_stack();

    if (cpu >= 0) {
      if (!set_affinity(cpu)) {
        std::cerr << "Continuing without CPU pin.\n";
      }
    }
    if (!set_rt_fifo(prio)) {
      std::cerr << "Failed to enable SCHED_FIFO. Are you running as root (sudo) or have CAP_SYS_NICE?\n";
      std::cerr << "Continuing without RT scheduling.\n";
    }
  } else {
    if (cpu >= 0) {
      if (!set_affinity(cpu)) {
        std::cerr << "Continuing without CPU pin.\n";
      }
    }
  }

  // Report actual scheduler
  int policy = sched_getscheduler(0);
  std::cout << "Scheduler policy: "
            << (policy == SCHED_FIFO ? "SCHED_FIFO" :
                policy == SCHED_RR   ? "SCHED_RR" :
                policy == SCHED_OTHER? "SCHED_OTHER" : "OTHER")
            << "\n";

  const long long period_ns = (long long)period_us * 1000LL;

  // schedule first tick = now + period
  timespec ts;
  clock_gettime(clk_id, &ts);
  ts = add_ns(ts, period_ns);

  Stats s{};
  std::vector<long long> samples;
  samples.reserve((size_t)duration_s * (1000000 / std::max(1, period_us)));

  auto end_time = std::chrono::steady_clock::now() + std::chrono::seconds(duration_s);

  long long cycles = 0;

  while (g_running.load() && std::chrono::steady_clock::now() < end_time) {
    // sleep to absolute tick
    int rc = clock_nanosleep(clk_id, TIMER_ABSTIME, &ts, nullptr);
    if (rc != 0 && rc != EINTR) {
      std::cerr << "clock_nanosleep error: " << std::strerror(rc) << "\n";
    }

    timespec now;
    clock_gettime(clk_id, &now);

    long long lateness_ns = diff_ns(now, ts);
    if (lateness_ns < 0) lateness_ns = 0;

    add_sample(s, lateness_ns);
    samples.push_back(lateness_ns);
    cycles++;

    // optional simulate work
    burn_some_cpu(burn_us);

    // schedule next tick
    ts = add_ns(ts, period_ns);

    // overrun handling: if we are behind more than one period, skip forward to next future tick
    timespec now2;
    clock_gettime(clk_id, &now2);
    while (diff_ns(now2, ts) > 0) {
      ts = add_ns(ts, period_ns);
    }

    if (print_every > 0 && (cycles % print_every) == 0) {
      long double avg = s.sum_ns / (long double)s.count;
      std::cout << "cycles=" << cycles
                << " avg_us=" << (double)(avg / 1000.0L)
                << " min_us=" << (double)s.min_ns / 1000.0
                << " max_us=" << (double)s.max_ns / 1000.0
                << "\n";
    }
  }

  std::cout << "\n=== Summary ===\n";
  print_summary(s, samples);

  return 0;
}