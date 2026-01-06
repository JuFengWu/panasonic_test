#include "panasonicA6B.hpp"

#include <iostream>
#include <memory>

int main()
{
  std::unique_ptr<Motor> motor = std::make_unique<PanasonicA6B>(1, PP_Mode);

  if (!motor->set_mode(PP_Mode)) {
    std::cout << "set_mode failed\n";
    return 1;
  }

  if (!motor->set_target_position(0.0f)) {
    std::cout << "set_target_position failed\n";
    return 1;
  }

  std::cout << "test_pp_mode ok\n";
  return 0;
}
