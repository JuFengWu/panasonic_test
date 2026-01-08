#pragma once

#include "motors.hpp"
#include "utilities.hpp"

class PanasonicA6B : public Motor {
 public:
  PanasonicA6B(int slave, MotorModes mode);

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
  int get_current_position() override;
  int get_current_velocity() override;
  int get_current_torque() override;
 MotorModes get_mode() override;

private:
  typedef enum {
    SWITCH_DISABLED,
    READY_SWITCH,
    SWITCHED_ON,
    OP_ENABLED,
    FAULT,
    UNKNOWN_PDS_STATE
  } PDSState;

  void dump_pdo(uint16 slave);
  PDSState getPDS(uint16 sw);
  static constexpr double kCountsPerRev = 0x800000;
  int32 deg_to_command(double deg);
  bool move_absolute_pp_pdo(uint16 slave, double target_deg);
  bool servoOnPDO_mapping4(uint16 slave);
  bool servoOffPDO_mapping4(uint16 slave);
};
