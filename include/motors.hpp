typedef enum {
    PP_Mode,
    CSP_Mode,
    CSV_Mode,
    CST_Mode,
    UNKNOWN
} Modes;


class Motor{
 public:
  Motor(int slave);
  virtual bool set_mode()=0;
  virtual bool set_target_position(float)=0;
  virtual bool set_target_velocity(float)=0;
  virtual bool set_target_torque(float)=0;
  virtual int get_error_code()=0;
  virtual int get_error_code()=0;
};