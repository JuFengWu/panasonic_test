#include "motionSystem.hpp"
#include <iostream>
#include <memory>
#include <unistd.h>

static MotorModes g_mode = CSP_Mode;

static bool run_mode_test(MotorModes mode, Motor& m1);

static void on_cycle(AllMotors& motors, bool& cycle_shutdown_request) {
  static int loop_count = 0;
  static int loops_since_servo_on = 0;
  static bool mode_done = false;
  ++loop_count;
  if (loop_count % 50 == 0) {
    auto currentPosition = motors.motor(1).get_current_position();
    std::cout << "Current Position: " << currentPosition << std::endl;
    auto errorState = motors.motor(1).get_error_code();
    std::cout << "Error State: " << static_cast<int>(errorState) << std::endl;
  }
  Motor& m1 = motors.motor(1);

  if (!mode_done) {
    loops_since_servo_on++;
    if (loops_since_servo_on >= 50) {
      if (run_mode_test(g_mode, m1)) {
        mode_done = true;
        printf("request shutdown\n");
        cycle_shutdown_request = true;
      }
    }
  }
}


static bool run_csp_mode_test_cycle(Motor& m1) {
  static bool initialized = false;
  static int cycle_index = 0;
  static int phase = 0;
  static int steps_remaining = 0;
  static int hold_ticks = 0;
  static float posA = 0.0f;
  static float posB = 30.0f;
  static float step = 0.0f;
  static float cmd = 0.0f;
  const int cycles = 2;
  const int total_steps = 500; // 2s / 4ms
  const int hold_ticks_total = 50; // 200ms / 4ms

  if (!initialized) {
    float posA_deg = 0.0f;
    float posB_deg = 30.0f;
    posA = posA_deg;
    posB = posB_deg;
    step = (posB - posA) / static_cast<float>(total_steps);
    if (step == 0.0f) {
      step = (posB > posA) ? 0.01f : -0.01f;
    }
    cycle_index = 0;
    phase = 0;
    steps_remaining = total_steps;
    hold_ticks = 0;
    cmd = posA;
    initialized = true;
    printf("==== CSP Cycle %d: A -> B ====\n", cycle_index + 1);
  }

  if (cycle_index >= cycles) {
    initialized = false;
    return true;
  }

  if (phase == 0) {
    m1.set_target_position(cmd);
    cmd += step;
    --steps_remaining;
    if (steps_remaining <= 0) {
      m1.set_target_position(posB);
      hold_ticks = hold_ticks_total;
      phase = 1;
    }
  } else if (phase == 1) {
    --hold_ticks;
    if (hold_ticks <= 0) {
      cmd = posB;
      steps_remaining = total_steps;
      phase = 2;
      printf("==== CSP Cycle %d: B -> A ====\n", cycle_index + 1);
    }
  } else if (phase == 2) {
    m1.set_target_position(cmd);
    cmd -= step;
    --steps_remaining;
    if (steps_remaining <= 0) {
      m1.set_target_position(posA);
      hold_ticks = hold_ticks_total;
      phase = 3;
    }
  } else if (phase == 3) {
    --hold_ticks;
    if (hold_ticks <= 0) {
      ++cycle_index;
      if (cycle_index >= cycles) {
        initialized = false;
        return true;
      }
      phase = 0;
      steps_remaining = total_steps;
      cmd = posA;
      printf("==== CSP Cycle %d: A -> B ====\n", cycle_index + 1);
    }
  }

  return false;
}

static bool run_csv_mode_test_cycle(Motor& m1, int32 vel_cmd, int cycles) {
  static bool initialized = false;
  static int cycle_index = 0;
  static int phase = 0;
  static int steps_remaining = 0;
  static int hold_ticks = 0;
  const int run_ms = 2000; // 每次跑 2 秒
  const int steps = run_ms / 4; // 2秒 / 4ms = 500 steps
  const int hold_ticks_total = 50; // 200ms / 4ms

  if (!initialized) {
    cycle_index = 0;
    phase = 0;
    steps_remaining = steps;
    hold_ticks = 0;
    initialized = true;
    printf("=== CSV Cycle %d: +vel %d for %dms ===\n", cycle_index + 1, vel_cmd, run_ms);
  }

  if (cycle_index >= cycles) {
    m1.set_target_velocity(0);
    initialized = false;
    return true;
  }

  if (phase == 0) {
    m1.set_target_velocity(vel_cmd);
    --steps_remaining;
    if (steps_remaining <= 0) {
      m1.set_target_velocity(0);
      hold_ticks = hold_ticks_total;
      phase = 1;
    }
  } else if (phase == 1) {
    --hold_ticks;
    if (hold_ticks <= 0) {
      steps_remaining = steps;
      phase = 2;
      printf("=== CSV Cycle %d: -vel %d for %dms ===\n", cycle_index + 1, vel_cmd, run_ms);
    }
  } else if (phase == 2) {
    m1.set_target_velocity(-vel_cmd);
    --steps_remaining;
    if (steps_remaining <= 0) {
      m1.set_target_velocity(0);
      hold_ticks = hold_ticks_total;
      phase = 3;
    }
  } else if (phase == 3) {
    --hold_ticks;
    if (hold_ticks <= 0) {
      ++cycle_index;
      if (cycle_index >= cycles) {
        m1.set_target_velocity(0);
        initialized = false;
        return true;
      }
      phase = 0;
      steps_remaining = steps;
      printf("=== CSV Cycle %d: +vel %d for %dms ===\n", cycle_index + 1, vel_cmd, run_ms);
    }
  }

  return false;
}

static bool run_cst_mode_test_cycle(Motor& m1, int16 tq_cmd, int cycles) {
  static bool initialized = false;
  static int cycle_index = 0;
  static int phase = 0;
  static int steps_remaining = 0;
  static int hold_ticks = 0;
  const int run_ms = 2000; // 每次跑 2 秒
  const int steps = run_ms / 4; // 2s / 4ms = 500 次更新
  const int hold_ticks_total = 50; // 200ms / 4ms

  if (!initialized) {
    cycle_index = 0;
    phase = 0;
    steps_remaining = steps;
    hold_ticks = 0;
    initialized = true;
    printf("=== CST Cycle %d: +Torque %d for %dms ===\n", cycle_index + 1, tq_cmd, run_ms);
  }

  if (cycle_index >= cycles) {
    m1.set_target_torque(0);
    initialized = false;
    return true;
  }

  if (phase == 0) {
    m1.set_target_torque(tq_cmd);
    --steps_remaining;
    if (steps_remaining <= 0) {
      m1.set_target_torque(0);
      hold_ticks = hold_ticks_total;
      phase = 1;
    }
  } else if (phase == 1) {
    --hold_ticks;
    if (hold_ticks <= 0) {
      steps_remaining = steps;
      phase = 2;
      printf("=== CST Cycle %d: -Torque %d for %dms ===\n", cycle_index + 1, tq_cmd, run_ms);
    }
  } else if (phase == 2) {
    m1.set_target_torque(-tq_cmd);
    --steps_remaining;
    if (steps_remaining <= 0) {
      m1.set_target_torque(0);
      hold_ticks = hold_ticks_total;
      phase = 3;
    }
  } else if (phase == 3) {
    --hold_ticks;
    if (hold_ticks <= 0) {
      ++cycle_index;
      if (cycle_index >= cycles) {
        m1.set_target_torque(0);
        initialized = false;
        return true;
      }
      phase = 0;
      steps_remaining = steps;
      printf("=== CST Cycle %d: +Torque %d for %dms ===\n", cycle_index + 1, tq_cmd, run_ms);
    }
  }

  return false;
}

static bool run_mode_test(MotorModes mode, Motor& m1) {
  switch (mode) {
    case PP_Mode:
      printf("Mode test not implemented yet.\n");
      return false;
      //return run_pp_mode_test(m1);
    case CSP_Mode:
      return run_csp_mode_test_cycle(m1);
    case CSV_Mode:
      return run_csv_mode_test_cycle(m1, 500000, 2);
    case CST_Mode:
      return run_cst_mode_test_cycle(m1, 10, 5);
    default:
      printf("Mode test not implemented yet.\n");
      return false;
  }
}

int main() {
  const char* ifname;

  ifname = "enp3s0";

  printf("使用介面卡: %s\n", ifname);

  MotionSystem sys(PanasonicA6MotorType, ifname);
  constexpr MotorModes kMode = CSP_Mode;// CSV have problem, may be unit is error?
  g_mode = kMode;
  int motor_count = 1;
  int cyclePeriod = 1; // in ms
  if (!sys.start_connect(motor_count,cyclePeriod, kMode)) {
    printf("sys.start_connect failed\n");
    return -1;
  }
  sys.set_cycle_log_enabled(true);

  // 取得 motors 與 session（使用者可見）
  auto& motors = sys.motors();
  auto& sess = sys.session();

  Motor& m1 = motors.motor(1);
  // Motor& m2 = motors.motor(2);

  printf("Motor count = %d\n", motors.count());

  //設定 realtime callback（callback 在 main 外面）
  sess.setCallback(on_cycle);

   if (!sys.run_async_io_only()) {
    printf("sys.run_async_io_only 失敗\n");
    sys.close();
    return -1;
  }
  m1.servo_on();
  sys.start_control_loop();
  //(void)m1;
  // m1.set_target_position(1000.0f);
  // m2.set_target_position(2000.0f);

  // stop: 停 thread + 回 SAFEOP
  // sys.stop();

  // close: ec_close + 清理資源
  m1.servo_off();
  std::cout << "Closing motion system..." << std::endl;
  sys.close();
  return 0;
}
