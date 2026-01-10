#include "motionSystem.hpp"
#include <iostream>
#include <memory>
#include <unistd.h>

static void on_cycle(AllMotors& motors, bool& break_loop) {

  auto currentPosition = motors.motor(1).get_current_position();
  std::cout << "Current Position: " << currentPosition << std::endl;
  auto errorState = motors.motor(1).get_error_code();
  std::cout << "Error State: " << static_cast<int>(errorState) << std::endl;
  (void)break_loop;
}

static bool run_pp_mode_test(Motor& m1){
  double posA = 0.0;
    double posB = 30.0;

    for (int i = 0; i < 5; i++)
    {
        printf("---- Cycle %d: move to %.1f deg ----\n", i+1, posB);
        if (!m1.set_target_position(posB))
        {
            printf("Move to %.1f deg failed\n", posB);
            break;
        }

        usleep(200000); // 200ms 停一下

        printf("---- Cycle %d: move to %.1f deg ----\n", i+1, posA);
        if (!m1.set_target_position(posA))
        {
            printf("Move to %.1f deg failed\n", posA);
            break;
        }

        usleep(200000);
    }

    printf("=== Move test done ===\n");
    return true;
}

static bool run_mode_test(MotorModes mode, Motor& m1)
{
  switch (mode) {
    case PP_Mode:
      return run_pp_mode_test(m1);
    default:
      printf("Mode test not implemented yet.\n");
      return false;
  }
}
int main(){

  const char* ifname;

  ifname = "enp3s0";

  printf("使用介面卡: %s\n", ifname);

  MotionSystem sys;
  constexpr MotorModes kMode = PP_Mode;

  if (!sys.start_connect(ifname, 1, FakeMotorType, kMode))
  {
      printf("sys.start_connect failed\n");
      return -1;
  }

  // 取得 motors 與 session（使用者可見）
  auto& motors = sys.motors();
  auto& sess   = sys.session();

  Motor& m1 = motors.motor(1); 
  //Motor& m2 = motors.motor(2);

  printf("Motor count = %d\n", motors.count());

  // ✅ 設定 realtime callback（callback 在 main 外面）
  sess.setCallback(on_cycle);

  // ✅ start: 內部進 OP + cyclicSession.start()
  if (!sys.run_async())
  {
      printf("sys.run_async 失敗\n");
      sys.close();
      return -1;
  }
  run_mode_test(kMode, m1);
  //m1.set_target_position(1000.0f);
  //m2.set_target_position(2000.0f);

  // ✅ stop: 停 thread + 回 SAFEOP
  //sys.stop();

  // ✅ close: ec_close + 清理資源
  sys.close();
  return 0;  
}
