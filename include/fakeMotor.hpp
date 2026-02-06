#pragma once

#include "motors.hpp"

class FakeMotor : public Motor {
 public:
  FakeMotor(int slave, MotorModes mode);

  bool set_mode(MotorModes mode) override;
  bool set_target_position(float target) override;
  bool set_target_velocity(float target) override;
  bool set_target_torque(float target) override;
  bool setMotionProfile(const MotionProfile& p) override;
  bool setVel(int vel) override;
  bool setAcc(int acc) override;
  bool setDec(int dec) override;
  bool servo_on() override;
  bool servo_off() override;
  int get_error_code() override;
  double get_current_position() override;
  double get_current_velocity() override;
  double get_current_torque() override;
  MotorModes get_mode() override;

 private:
  int current_position_;
  int current_velocity_;
  int current_torque_;
  int error_code_;
  MotionProfile profile_;
  float target_position_;
  float target_velocity_;
  float target_torque_;
};
