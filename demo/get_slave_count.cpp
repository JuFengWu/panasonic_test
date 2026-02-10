
#include "Ethercat.hpp"
#include "motionSystem.hpp"
#include <iostream>
#include <memory>
int main() {
  const char* ifname;

  ifname = "enp3s0";

  printf("使用介面卡: %s\n", ifname);

  MotionSystem sys(PanasonicA6MotorType, ifname);

  int count = 0;
  if (!sys.get_slave_count(count)) {
    printf("get_slave_count failed\n");
    return -1;
  }
  printf("slave count = %d\n", count);
}
