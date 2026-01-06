#pragma once

#include "motors.hpp"

class PanasonicA6B : public Motor {
 public:
  explicit PanasonicA6B(int slave);

  bool set_mode() override;
  bool set_target_position(float target) override;
  bool set_target_velocity(float target) override;
  bool set_target_torque(float target) override;
  bool setMotionProfile(const MotionProfile& p) override;
  bool setVel(int vel) override;
  bool setAcc(int acc) override;
  bool setDec(int dec) override;
  int get_error_code() override;
  int get_current_position() override;
  int get_current_velocity() override;
  int get_current_torque() override;

 private:
  int slave_;
};
