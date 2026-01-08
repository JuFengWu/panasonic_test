#pragma once
#include <functional>
#include <utility>
typedef enum {
    PP_Mode,
    CSP_Mode,
    CSV_Mode,
    CST_Mode,
    UNKNOWN
} MotorModes;

typedef enum {
    PanasonicA6BMotorType,
    FakeMotorType,
    UnknownMotorType
} MotorModel;

struct MotionProfile {
    int vel;
    int acc;
    int dec;
};

class Motor{
 public:
  Motor(int slave, MotorModes mode);
  using FatalHandler = std::function<void()>;
  void set_fatal_handler(FatalHandler handler) { fatal_handler_ = std::move(handler); }
  virtual bool set_mode(MotorModes mode)=0;
  virtual bool set_target_position(float)=0;
  virtual bool set_target_velocity(float)=0;
  virtual bool set_target_torque(float)=0;
  virtual bool setMotionProfile(const MotionProfile& p)=0;
  virtual bool setVel(int vel)=0;
  virtual bool setAcc(int acc)=0;
  virtual bool setDec(int dec)=0;
  virtual bool servo_on()=0;
  virtual bool servo_off()=0;
  virtual int get_error_code()=0;
  virtual int get_current_position()=0;
  virtual int get_current_velocity()=0;
  virtual int get_current_torque()=0;
  virtual MotorModes get_mode()=0;

 protected:
  int slave_;
  MotorModes mode_;
  void notify_fatal() { if (fatal_handler_) fatal_handler_(); }

 private:
  FatalHandler fatal_handler_;
};
