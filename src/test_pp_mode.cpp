#include "panasonicA6B.hpp"

#include <iostream>

int main()
{
  PanasonicA6B motor(1);

  if (!motor.set_mode()) {
    std::cout << "set_mode failed\n";
    return 1;
  }

  if (!motor.set_target_position(0.0f)) {
    std::cout << "set_target_position failed\n";
    return 1;
  }

  std::cout << "test_pp_mode ok\n";
  return 0;
}
