#pragma once

#include "motors.hpp"
#include "utilities.hpp"

class PanasonicA6 : public Motor {
 public:
  PanasonicA6(int slave, MotorModes mode);

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
  void init_csv_mode(uint16 slave);
  void init_cst_mode(uint16 slave);
  void csv_set_target_velocity(uint16 slave, int32 vel_cmd);
  void csp_set_target_position(uint16 slave, float target_degree);
  bool servoOnPDO_mapping4(uint16 slave);
  bool servoOffPDO_mapping4(uint16 slave);
};
/*
    SET PDO maping 4   
			Index	  Size(bit)	Name
    RxPDO (1603h)	6040h 00h 16 Controlword
			6060h 00h  8 Modes of operation
			6071h 00h 16 Target Torque
			6072h 00h 16 Max torque
			607Ah 00h 32 Target Position
			6080h 00h 32 Max motor speed
			60B8h 00h 16 Touch probe function
			60FFh 00h 32 Target Velocity
    TxPDO (1A03h)
			603Fh 00h 16 Error code
			6041h 00h 16 Statusword
			6061h 00h  8 Modes of operation display
			6064h 00h 32 Position actual value
			606Ch 00h 32 Velocity actual value
			6077h 00h 16 Torque actual value
			60B9h 00h 16 Touch probe status
			60BAh 00h 32 Touch probe pos1 pos val
			60FDh 00h 32 Digital inputs
   */


