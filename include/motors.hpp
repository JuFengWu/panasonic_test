typedef enum {
    PP_Mode,
    CSP_Mode,
    CSV_Mode,
    CST_Mode,
    UNKNOWN
} Modes;

struct MotionProfile {
    int vel;
    int acc;
    int dec;
};

class Motor{
 public:
  Motor(int slave);
  virtual bool set_mode()=0;
  virtual bool set_target_position(float)=0;
  virtual bool set_target_velocity(float)=0;
  virtual bool set_target_torque(float)=0;
  virtual bool setMotionProfile(const MotionProfile& p)=0;
  virtual bool setVel(int vel)=0;
  virtual bool setAcc(int acc)=0;
  virtual bool setDec(int dec)=0;
  virtual int get_error_code()=0;
  virtual int get_current_position()=0;
  virtual int get_current_velocity()=0;
  virtual int get_current_torque()=0;
};
