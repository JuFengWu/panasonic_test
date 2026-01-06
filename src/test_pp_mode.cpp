#include "panasonicA6B.hpp"
#include "cyclicSession.hpp"
#include <iostream>
#include <memory>

static void on_cycle(Motors& motors, bool& break_loop) {
  auto* t = static_cast<double*>(user);
  *t += dt_sec;

  auto currentPosition = motors[0].position();
  std::cout << "Current Position: " << currentPosition << std::endl;
  auto errorState = motors.get_id(1).error_state();
  std::cout << "Error State: " << static_cast<int>(errorState) << std::endl;
  (void)break_loop;
}

int main(){

  const char* ifname;

  ifname = "enp3s0";

  printf("使用介面卡: %s\n", ifname);

  MotionSystem sys;

  // ✅ open: 內部完成掃描/config/map/SAFEOP SDO init（此時不啟動 cyclic thread）
  if (!sys.open(ifname))
  {
      printf("sys.open 失敗\n");
      return -1;
  }

  // 取得 motors 與 session（使用者可見）
  auto& motors = sys.motors(PP_Mode);
  auto& sess   = sys.session();

  Motor& m1 = motors.motor(1); 
  Motor& m2 = motors.motor(2);

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
  m1.set_target_position(1000.0f);
  m2.set_target_position(2000.0f);

  // ✅ stop: 停 thread + 回 SAFEOP
  sys.stop();

  // ✅ close: ec_close + 清理資源
  sys.close();
  return 0;  
}