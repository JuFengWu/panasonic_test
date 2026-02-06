#include "panasonicA6.hpp"
#include <iostream>
#include <unistd.h>

Motor::Motor(int slave, MotorModes mode) : slave_(slave), mode_(mode) {}

PanasonicA6::PanasonicA6(int slave, MotorModes mode) : Motor(slave, mode) {}

void PanasonicA6::init_csv_mode(uint16 slave) {
  uint8 *out = (uint8*)ec_slave[slave].outputs;

  // 6060 Mode = 9 (CSV)
  out[2] = 9;

  // Max torque (6072) 設大，避免出力被限制
  set_u16(out, 5, 1000); // 100% (你之前用 1000 OK)

  // Max motor speed (6080) 設合理（你之前用 0x16000000 OK）
  set_i32(out, 11, 0x16000000);

  // Target velocity = 0
  set_i32(out, 17, 0);

  // Controlword = 0x000F (Enable operation)
  set_u16(out, 0, 0x000F);
}

void PanasonicA6::init_cst_mode(uint16 slave) {
  uint8 *out = (uint8*)ec_slave[slave].outputs;

  // 6060 operation mode = 10 (CST)
  out[2] = 10;

  // 6072 max torque (先設定大一點)
  // 你前面用 1000 OK（多數 Panasonic 是 0.1% 或 1% 的單位）
  set_u16(out, 5, 1000);

  // 6071 target torque 初始 = 0
  set_i16(out, 3, 0);

  // 6080 max motor speed（強烈建議設，不然 torque mode 會無限制加速）
  set_i32(out, 11, 0x16000000);

  // controlword = 0x000F 保持 enable
  set_u16(out, 0, 0x000F);
}

bool PanasonicA6::set_mode(MotorModes mode) {
  mode_ = mode;
  if (mode_ == CSV_Mode) {
    init_csv_mode(slave_);
  } else if (mode_ == CST_Mode) {
    init_cst_mode(slave_);
  }

  return true;
}

// 假設：電子齒輪設定為 1，且「1 指令單位 = 1 度」。
int32 PanasonicA6::deg_to_command(double deg) {
  return (int32)(deg / 360.0 * kCountsPerRev);
}

bool PanasonicA6::move_absolute_pp_pdo(uint16 slave, double target_deg) {
  uint8 *out = (uint8*)ec_slave[slave].outputs;
  uint8 *in = (uint8*)ec_slave[slave].inputs;

  int32 target_cmd = deg_to_command(target_deg);

  printf("=== Before move ===\n");
  // dump_pdo(slave);

  // 0) 等 bit10 先變 0（確保下一次會重新變 1）
  int t = 0;
  while (t < 1000) {
    uint16 sw = get_u16(in, 2);
    if (!(sw & (1 << 10))) {
      break;
    }
    usleep(1000);
    t += 1;
  }

  // 1) 寫 target position (你目前用 offset=7)
  set_i32(out, 7, target_cmd);

  // 2) 控制字準備
  uint16 cw = get_u16(out, 0);
  cw |= 0x000F; // enable
  cw &= ~(1 << 6); // absolute

  // 先清 bit4
  cw &= ~(1 << 4);
  set_u16(out, 0, cw);
  usleep(2000);

  // 再 set bit4
  cw |= (1 << 4);
  set_u16(out, 0, cw);

  // 3) 等 bit12 ack = 1
  t = 0;
  while (t < 2000) {
    uint16 sw = get_u16(in, 2);
    if (sw & (1 << 12)) {
      break;
    }
    usleep(1000);
    t += 1;
  }

  // 4) 等 bit10 reached = 1 + debug current position
  int timeout_ms = 5000;
  int elapsed = 0;
  int next_print = 0;

  while (elapsed < timeout_ms) {
    uint16 sw = get_u16(in, 2);
    int32 act = get_i32(in, 5); // 先用 5，但我們用 dump 來確認

    if (elapsed >= next_print) {
      printf("[move dbg] t=%4dms sw=0x%04X act=%d target=%d diff=%d\n",
             elapsed, sw, act, target_cmd, target_cmd - act);
      next_print += 100;
    }

    if (sw & (1 << 10)) {
      printf("[move] reached target=%.2f deg cmd=%d sw=0x%04X act=%d diff=%d\n",
             target_deg, target_cmd, sw, act, target_cmd - act);

      printf("=== After reached ===\n");
      dump_pdo(slave);

      // 清 bit4（讓下次 move toggle 有效）
      cw &= ~(1 << 4);
      set_u16(out, 0, cw);

      return true;
    }

    usleep(10000);
    elapsed += 10;
  }

  uint16 sw = get_u16(in, 2);
  int32 act = get_i32(in, 5);

  printf("[move] timeout! target=%.2f cmd=%d sw=0x%04X act=%d diff=%d\n",
         target_deg, target_cmd, sw, act, target_cmd - act);

  printf("=== After timeout ===\n");
  dump_pdo(slave);

  // 清 bit4
  cw &= ~(1 << 4);
  set_u16(out, 0, cw);

  return false;
}

void PanasonicA6::csp_set_target_position(uint16 slave, float target_degree) {
  int32 target_cmd = deg_to_command(target_degree);
  uint8 *out = (uint8*)ec_slave[slave].outputs;

  // CSP: 只要一直更新 607A (offset=7)
  set_i32(out, 7, target_cmd);
}

bool PanasonicA6::set_target_position(float target) {
  (void)target;
  if (mode_ == PP_Mode) {
    return move_absolute_pp_pdo(slave_, target);
  } else if (mode_ == CSP_Mode) {
    csp_set_target_position(slave_, target);
    return true;
  }
  return false;
}

void PanasonicA6::csv_set_target_velocity(uint16 slave, int32 vel_cmd) {
  uint8 *out = (uint8*)ec_slave[slave].outputs;
  set_i32(out, 17, vel_cmd); // 60FF offset=17
}

inline void PanasonicA6::cst_set_target_torque(uint16 slave, int16 tq_cmd) {
  uint8 *out = (uint8*)ec_slave[slave].outputs;

  set_i16(out, 3, tq_cmd); // 6071 offset=3
}
inline double PanasonicA6::cmdps_to_rpm(int32_t cmd_per_sec)
{
    const double cmd_per_rev = 8388608.0; // 6092:01 / 6092:02 (= 608F 23bit/r)
    return (cmd_per_sec * 60.0) / cmd_per_rev;
}

inline int32_t PanasonicA6::rpm_to_cmdps(double rpm)
{
    const double cmd_per_rev = 8388608.0;
    return (int32_t)((rpm * cmd_per_rev) / 60.0);
}
inline uint32_t PanasonicA6::rpmps_to_cmdps2(double rpm_per_sec)
{
    const double cmd_per_rev = 8388608.0;

    double cmdps2 = (rpm_per_sec / 60.0) * cmd_per_rev;

    if (cmdps2 < 0) cmdps2 = 0;
    if (cmdps2 > 4294967295.0) cmdps2 = 4294967295.0;

    return (uint32_t)(cmdps2 + 0.5);
}
bool PanasonicA6::set_target_velocity(float target) {
  int32 cmdps = rpm_to_cmdps(target);
  csv_set_target_velocity(slave_, cmdps);
  return true;
}

bool PanasonicA6::set_target_torque(float target) {
  cst_set_target_torque(slave_, (int16)(target * 10)); // panasonic 6071 單位是 0.1Nm
  return false;
}

bool PanasonicA6::setMotionProfile(const MotionProfile& p) {
  return setVel(p.vel) && setAcc(p.acc) && setDec(p.dec);
}

bool PanasonicA6::setVel(int vel) {
    
  uint32_t vel_cmdps = rpm_to_cmdps(vel);
  int ok = 1;
  ok &= sdo_write_u32(slave_, 0x6081, 0x00, vel_cmdps);
  printf("Set 6081 vel=%d rpm -> %u cmd/s\n", vel, vel_cmdps);
  if (!ok) {
    return false;
  }
  return true;
}

bool PanasonicA6::setAcc(int acc) {
  uint32_t acc_cmd = rpmps_to_cmdps2(acc);

  int ok = 1;
  ok &= sdo_write_u32(slave_, 0x6083, 0x00, acc_cmd);

  printf("Set acc = %d rpm/s -> %u cmd/s^2\n",
       acc, acc_cmd);
  if (!ok) {
    return false;
  }
  return true;
}

bool PanasonicA6::setDec(int dec) {
  uint32_t dec_cmd = rpmps_to_cmdps2(dec);

  int ok = 1;
  ok &= sdo_write_u32(slave_, 0x6084, 0x00, dec_cmd);

  printf("Set dec = %d rpm/s -> %u cmd/s^2\n",
       dec, dec_cmd);
  if (!ok) {
    return false;
  }
  return true;
}

void PanasonicA6::dump_pdo(uint16 slave) {
  printf("==== PDO DUMP slave %d ====\n", slave);

  printf("Obytes=%d: ", ec_slave[slave].Obytes);
  for (int i = 0; i < ec_slave[slave].Obytes; i++) {
    printf("%02X ", ((uint8*)ec_slave[slave].outputs)[i]);
  }
  printf("\n");

  printf("Ibytes=%d: ", ec_slave[slave].Ibytes);
  for (int i = 0; i < ec_slave[slave].Ibytes; i++) {
    printf("%02X ", ((uint8*)ec_slave[slave].inputs)[i]);
  }
  printf("\n");
  printf("==========================\n");
}

PanasonicA6::PDSState PanasonicA6::getPDS(uint16 sw) {
  if ((sw & 0x004F) == 0x0040) {
    return SWITCH_DISABLED;
  }
  if ((sw & 0x006F) == 0x0021) {
    return READY_SWITCH;
  }
  if ((sw & 0x006F) == 0x0023) {
    return SWITCHED_ON;
  }
  if ((sw & 0x006F) == 0x0027) {
    return OP_ENABLED;
  }
  if ((sw & 0x004F) == 0x0008) {
    return FAULT;
  }
  return UNKNOWN_PDS_STATE;
}

bool PanasonicA6::servoOnPDO_mapping4(uint16 slave) {
  uint8 *out = (uint8*)ec_slave[slave].outputs; // 25 bytes
  uint8 *in = (uint8*)ec_slave[slave].inputs; // 25 bytes

  // ? mapping4 下，通常 outputs[0..1] = controlword
  // ? outputs[2] = mode (6060)
  if (mode_ == PP_Mode) {
    out[2] = 1; // PP mode
  } else if (mode_ == CSV_Mode) {
    out[2] = 9; // CSV mode
  } else if (mode_ == CST_Mode) {
    out[2] = 10; // CST mode
  } else if (mode_ == CSP_Mode) {
    out[2] = 8; // CSP mode
  } else {
    out[2] = 1; // PP mode
  }

  int loop = 0;
  while (1) {
    uint16 sw = get_u16(in, 2); // ? statusword 仍是 offset=2 （你的 dump 保證）
    PDSState st = getPDS(sw);

    if (st == OP_ENABLED) {
      printf("[servoOn] OPERATION_ENABLED sw=0x%04X\n", sw);
      return true;
    }

    uint16 cw = 0x0006;
    switch (st) {
      case SWITCH_DISABLED:
        cw = 0x0006;
        break;
      case READY_SWITCH:
        cw = 0x0007;
        break;
      case SWITCHED_ON:
        cw = 0x000F;
        break;
      case FAULT:
        cw = 0x0080;
        break;
      default:
        cw = 0x0006;
        break;
    }

    set_u16(out, 0, cw); // ? 改成 offset=0 !!!

    usleep(10000);

    if (loop++ % 50 == 0) {
      printf("[servoOn] sw=0x%04X st=%d cw=0x%04X modeDisp=%d\n",
             sw, st, cw, in[4]);
      dump_pdo(slave);
    }
  }
}

bool PanasonicA6::servo_on() {
  printf("開始 Servo ON...\n");
  if (!servoOnPDO_mapping4(slave_)) {
    notify_fatal();
    return false;
  }
  printf("Servo ON 完成\n");
  return true;
}

bool PanasonicA6::servoOffPDO_mapping4(uint16 slave) {
  uint8 *out = (uint8*)ec_slave[slave].outputs;
  uint8 *in = (uint8*)ec_slave[slave].inputs;

  int loop = 0;

  while (1) {
    uint16 sw = get_u16(in, 2); // statusword offset=2
    PDSState st = getPDS(sw);

    // 目標：回到 Switch Disabled
    if (st == SWITCH_DISABLED) {
      printf("[servoOff] SWITCH_DISABLED sw=0x%04X\n", sw);
      return true;
    }

    uint16 cw = 0x0000;

    switch (st) {
      case OP_ENABLED:
        cw = 0x0007; // disable operation → 回到 switched on
        break;

      case SWITCHED_ON:
        cw = 0x0006; // shutdown → 回到 ready to switch on
        break;

      case READY_SWITCH:
        cw = 0x0000; // disable voltage → 回到 switch disabled
        break;

      case FAULT:
        // fault 狀態下 servo off 沒意義，給 fault reset
        cw = 0x0080;
        break;

      default:
        cw = 0x0000;
        break;
    }

    set_u16(out, 0, cw); // ? controlword offset=0
    usleep(10000); // 10ms

    if (loop++ % 50 == 0) {
      uint16 err = get_u16(in, 0); // 603F error code
      printf("[servoOff] sw=0x%04X st=%d cw=0x%04X err=0x%04X\n",
             sw, st, cw, err);
    } // TODO?  retun false if too long??
  }
}

bool PanasonicA6::servo_off() {
  return servoOffPDO_mapping4(slave_);
}

int PanasonicA6::get_error_code() {
  //uint8 *in = (uint8*)ec_slave[slave_].inputs;
  //return (int)get_u16(in, 0); // 603F:00 Error code

  uint8 *in = (uint8*)ec_slave[slave_].inputs;

  // 603F:00 (TxPDO offset 0)
  uint16_t code = get_u16(in, 0);       // e.g. 0xFF50
  int main_code = code & 0x00FF;        // 80

  return main_code;
}

double PanasonicA6::get_current_position() {
  uint8 *in = (uint8*)ec_slave[slave_].inputs;   // pulse
  int32 pulse = get_i32(in, 5); // 6064:00 Position actual value
  double deg = static_cast<double>(pulse) * 360.0 / kCountsPerRev;
  return deg;
}

double PanasonicA6::get_current_velocity() {
  uint8 *in = (uint8*)ec_slave[slave_].inputs; // cmd/s
  int32 cmdps = get_i32(in, 9); // 606C:00 Velocity actual value
  return cmdps_to_rpm(cmdps);
}

double PanasonicA6::get_current_torque() {
  uint8 *in = (uint8*)ec_slave[slave_].inputs;  // 0.1% rated torque
  int16 raw = get_i16(in, 13); // 6077:00 Torque actual value
  return static_cast<double>(raw) / 10.0;
}
MotorModes PanasonicA6::get_mode() { return mode_; }
