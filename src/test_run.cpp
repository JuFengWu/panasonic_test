#include "motionSystem.hpp"
#include <iostream>
#include <memory>
#include <unistd.h>

static void on_cycle(AllMotors& motors, bool& break_loop) {
  //auto currentPosition = motors.motor(1).get_current_position();
  //std::cout << "Current Position: " << currentPosition << std::endl;
  //auto errorState = motors.motor(1).get_error_code();
  //std::cout << "Error State: " << static_cast<int>(errorState) << std::endl;
  //(void)break_loop;
}

static bool run_pp_mode_test(Motor& m1) {
  double posA = 0.0;
  double posB = 30.0;

  for (int i = 0; i < 2; i++) {
    printf("---- Cycle %d: move to %.1f deg ----\n", i + 1, posB);
    if (!m1.set_target_position(posB)) {
      printf("Move to %.1f deg failed\n", posB);
      break;
    }

    usleep(200000); // 200ms 停一下

    printf("---- Cycle %d: move to %.1f deg ----\n", i + 1, posA);
    if (!m1.set_target_position(posA)) {
      printf("Move to %.1f deg failed\n", posA);
      break;
    }

    usleep(200000);
  }

  printf("=== Move test done ===\n");
  return true;
}

static bool run_csp_mode_test(Motor& m1) {
  float posA_deg = 0.0f;
  float posB_deg = 30.0f;
  int cycles = 5;
  float posA = posA_deg;
  float posB = posB_deg;

  // 每 4ms 更新一次（配合你的 PDO thread 週期）
  const int dt_us = 4000;

  // 用一個合理速度，例如 30 度要 2 秒到達
  // step = 每週期增加的 command
  int total_steps = 2000.0 / 4.0; // 2秒 / 4ms = 500 steps
  float step = (posB - posA) / total_steps;
  if (step == 0.0f) {
    step = (posB > posA) ? 0.01f : -0.01f;
  }

  for (int c = 0; c < cycles; c++) {
    printf("==== CSP Cycle %d: A -> B ====\n", c + 1);

    // A -> B
    float cmd = posA;
    while ((step > 0 && cmd < posB) || (step < 0 && cmd > posB)) {
      m1.set_target_position(cmd);
      cmd += step;
      usleep(dt_us);
    }
    m1.set_target_position(posB);

    usleep(200000); // 停 200ms

    printf("==== CSP Cycle %d: B -> A ====\n", c + 1);

    // B -> A
    cmd = posB;
    while ((step > 0 && cmd > posA) || (step < 0 && cmd < posA)) {
      m1.set_target_position(cmd);
      cmd -= step;
      usleep(dt_us);
    }
    m1.set_target_position(posA);

    usleep(200000);
  }
  return true;
}

void csv_velocity_test(Motor& m1, int32 vel_cmd, int cycles) {
  const int dt_us = 4000; // 4ms
  const int run_ms = 2000; // 每次跑 2 秒
  const int steps = run_ms / 4; // 2秒 / 4ms = 500 steps

  for (int c = 0; c < cycles; c++) {
    printf("=== CSV Cycle %d: +vel %d for %dms ===\n", c + 1, vel_cmd, run_ms);

    for (int i = 0; i < steps; i++) {
      m1.set_target_velocity(vel_cmd);
      usleep(dt_us);
    }

    // 停下來
    m1.set_target_velocity(0);
    usleep(200000);

    printf("=== CSV Cycle %d: -vel %d for %dms ===\n", c + 1, vel_cmd, run_ms);

    for (int i = 0; i < steps; i++) {
      m1.set_target_velocity(-vel_cmd);
      usleep(dt_us);
    }

    // 停下來
    m1.set_target_velocity(0);
    usleep(200000);
  }

  // 最後確保停止
  m1.set_target_velocity(0);
}

void cst_torque_test(Motor& m1, int16 tq_cmd, int cycles) {
  const int dt_us = 4000; // 4ms
  const int run_ms = 2000; // 每次跑 2 秒
  const int steps = run_ms / 4; // 2s / 4ms = 500 次更新

  for (int c = 0; c < cycles; c++) {
    printf("=== CST Cycle %d: +Torque %d for %dms ===\n", c + 1, tq_cmd, run_ms);
    for (int i = 0; i < steps; i++) {
      m1.set_target_torque(tq_cmd);
      usleep(dt_us);
    }

    // torque = 0
    m1.set_target_torque(0);
    usleep(200000);

    printf("=== CST Cycle %d: -Torque %d for %dms ===\n", c + 1, tq_cmd, run_ms);
    for (int i = 0; i < steps; i++) {
      m1.set_target_torque(-tq_cmd);
      usleep(dt_us);
    }

    // torque = 0
    m1.set_target_torque(0);
    usleep(200000);
  }

  // 最後一定要歸 0 扭矩
  m1.set_target_torque(0);
}

static bool run_mode_test(MotorModes mode, Motor& m1) {
  switch (mode) {
    case PP_Mode:
      return run_pp_mode_test(m1);
    case CSP_Mode:
      return run_csp_mode_test(m1);
    case CSV_Mode:
      csv_velocity_test(m1, 500000, 5);
      return true;
    case CST_Mode:
      cst_torque_test(m1, 10, 5);
      return true;
    default:
      printf("Mode test not implemented yet.\n");
      return false;
  }
}

int main() {
  const char* ifname;

  ifname = "enp3s0";

  printf("使用介面卡: %s\n", ifname);

  MotionSystem sys;
  constexpr MotorModes kMode = CST_Mode;// CSV have problem, may be unit is error?

  if (!sys.start_connect(ifname, 1, PanasonicA6MotorType, kMode)) {
    printf("sys.start_connect failed\n");
    return -1;
  }

  // 取得 motors 與 session（使用者可見）
  auto& motors = sys.motors();
  auto& sess = sys.session();

  Motor& m1 = motors.motor(1);
  // Motor& m2 = motors.motor(2);

  printf("Motor count = %d\n", motors.count());

  // ? 設定 realtime callback（callback 在 main 外面）
  sess.setCallback(on_cycle);

  // ? start: 內部進 OP + cyclicSession.start()
  if (!sys.run_async()) {
    printf("sys.run_async 失敗\n");
    sys.close();
    return -1;
  }
  m1.servo_on();
  printf("start servo on!\n");
  run_mode_test(kMode, m1);
  printf("start do servo off!");
  m1.servo_off();
  printf("do servo off!");
  // m1.set_target_position(1000.0f);
  // m2.set_target_position(2000.0f);

  // stop: 停 thread + 回 SAFEOP
  // sys.stop();

  // close: ec_close + 清理資源
  sys.close();
  return 0;
}
