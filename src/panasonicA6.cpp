#include "panasonicA6.hpp"
#include "Ethercat.hpp"

#include <iostream>
#include <unistd.h>
#include <utility>

namespace {
inline void a6_set_i16(uint8 *p, int off, int16 v) {
  p[off] = static_cast<uint8>(v & 0xFF);
  p[off + 1] = static_cast<uint8>((v >> 8) & 0xFF);
}

inline int16 a6_get_i16(const uint8 *p, int off) {
  return static_cast<int16>(p[off] | (p[off + 1] << 8));
}

inline uint16 a6_get_u16(const uint8 *p, int off) {
  return static_cast<uint16>(p[off] | (p[off + 1] << 8));
}

inline void a6_set_u16(uint8 *p, int off, uint16 v) {
  p[off] = static_cast<uint8>(v & 0xFF);
  p[off + 1] = static_cast<uint8>((v >> 8) & 0xFF);
}

inline int32 a6_get_i32(const uint8 *p, int off) {
  return static_cast<int32>(p[off] | (p[off + 1] << 8) | (p[off + 2] << 16) |
                            (p[off + 3] << 24));
}

inline void a6_set_i32(uint8 *p, int off, int32 v) {
  p[off] = static_cast<uint8>(v & 0xFF);
  p[off + 1] = static_cast<uint8>((v >> 8) & 0xFF);
  p[off + 2] = static_cast<uint8>((v >> 16) & 0xFF);
  p[off + 3] = static_cast<uint8>((v >> 24) & 0xFF);
}
} // namespace

Motor::Motor(int slave, MotorModes mode, std::shared_ptr<MyEthercat> ethercat)
    : slave_(slave), mode_(mode), ethercat_(std::move(ethercat)) {}

PanasonicA6::PanasonicA6(int slave, MotorModes mode, std::shared_ptr<MyEthercat> ethercat)
    : Motor(slave, mode, std::move(ethercat)) {}

void PanasonicA6::init_csv_mode(uint16 slave) {
  if (!ethercat_) {
    return;
  }
  uint8 *out = ethercat_->get_slave_outputs(slave);
  if (!out) {
    return;
  }

  // 6060 Mode = 9 (CSV)
  out[2] = 9;

  // Max torque (6072) �]�j�A�קK�X�O�Q����
  a6_set_u16(out, 5, 1000); // 100% (�A���e�� 1000 OK)

  // Max motor speed (6080) �]�X�z�]�A���e�� 0x16000000 OK�^
  a6_set_i32(out, 11, 0x16000000);

  // Target velocity = 0
  a6_set_i32(out, 17, 0);

  // Controlword = 0x000F (Enable operation)
  a6_set_u16(out, 0, 0x000F);
}

void PanasonicA6::init_cst_mode(uint16 slave) {
  if (!ethercat_) {
    return;
  }
  uint8 *out = ethercat_->get_slave_outputs(slave);
  if (!out) {
    return;
  }

  // 6060 operation mode = 10 (CST)
  out[2] = 10;

  // 6072 max torque (���]�w�j�@�I)
  // �A�e���� 1000 OK�]�h�� Panasonic �O 0.1% �� 1% �����^
  a6_set_u16(out, 5, 1000);

  // 6071 target torque ��l = 0
  a6_set_i16(out, 3, 0);

  // 6080 max motor speed�]�j�P��ĳ�]�A���M torque mode �|�L����[�t�^
  a6_set_i32(out, 11, 0x16000000);

  // controlword = 0x000F �O�� enable
  a6_set_u16(out, 0, 0x000F);
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

// ���]�G�q�l�����]�w�� 1�A�B�u1 ���O��� = 1 �סv�C
int32 PanasonicA6::deg_to_command(double deg) {
  return (int32)(deg / 360.0 * kCountsPerRev);
}

bool PanasonicA6::move_absolute_pp_pdo(uint16 slave, double target_deg) {
  if (!ethercat_) {
    return false;
  }
  uint8 *out = ethercat_->get_slave_outputs(slave);
  const uint8 *in = ethercat_->get_slave_inputs(slave);
  if (!out || !in) {
    return false;
  }

  int32 target_cmd = deg_to_command(target_deg);

  printf("=== Before move ===\n");
  // dump_pdo(slave);

  // 0) �� bit10 ���� 0�]�T�O�U�@���|���s�� 1�^
  int t = 0;
  while (t < 1000) {
    uint16 sw = a6_get_u16(in, 2);
    if (!(sw & (1 << 10))) {
      break;
    }
    usleep(1000);
    t += 1;
  }

  // 1) �g target position (�A�ثe�� offset=7)
  a6_set_i32(out, 7, target_cmd);

  // 2) ����r�ǳ�
  uint16 cw = a6_get_u16(out, 0);
  cw |= 0x000F; // enable
  cw &= ~(1 << 6); // absolute

  // ���M bit4
  cw &= ~(1 << 4);
  a6_set_u16(out, 0, cw);
  usleep(2000);

  // �A set bit4
  cw |= (1 << 4);
  a6_set_u16(out, 0, cw);

  // 3) �� bit12 ack = 1
  t = 0;
  while (t < 2000) {
    uint16 sw = a6_get_u16(in, 2);
    if (sw & (1 << 12)) {
      break;
    }
    usleep(1000);
    t += 1;
  }

  // 4) �� bit10 reached = 1 + debug current position
  int timeout_ms = 5000;
  int elapsed = 0;
  int next_print = 0;

  while (elapsed < timeout_ms) {
    uint16 sw = a6_get_u16(in, 2);
    int32 act = a6_get_i32(in, 5); // ���� 5�A���ڭ̥� dump �ӽT�{

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

      // �M bit4�]���U�� move toggle ���ġ^
      cw &= ~(1 << 4);
      a6_set_u16(out, 0, cw);

      return true;
    }

    usleep(10000);
    elapsed += 10;
  }

  uint16 sw = a6_get_u16(in, 2);
  int32 act = a6_get_i32(in, 5);

  printf("[move] timeout! target=%.2f cmd=%d sw=0x%04X act=%d diff=%d\n",
         target_deg, target_cmd, sw, act, target_cmd - act);

  printf("=== After timeout ===\n");
  dump_pdo(slave);

  // �M bit4
  cw &= ~(1 << 4);
  a6_set_u16(out, 0, cw);

  return false;
}

void PanasonicA6::csp_set_target_position(uint16 slave, float target_degree) {
  if (!ethercat_) {
    return;
  }
  uint8 *out = ethercat_->get_slave_outputs(slave);
  if (!out) {
    return;
  }
  int32 target_cmd = deg_to_command(target_degree);

  // CSP: 607A (offset=7)
  a6_set_i32(out, 7, target_cmd);
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
  if (!ethercat_) {
    return;
  }
  uint8 *out = ethercat_->get_slave_outputs(slave);
  if (!out) {
    return;
  }
  a6_set_i32(out, 17, vel_cmd); // 60FF offset=17
}

void PanasonicA6::cst_set_target_torque(uint16 slave, int16 tq_cmd) {
  if (!ethercat_) {
    return;
  }
  uint8 *out = ethercat_->get_slave_outputs(slave);
  if (!out) {
    return;
  }

  a6_set_i16(out, 3, tq_cmd); // 6071 offset=3
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
  cst_set_target_torque(slave_, (int16)(target * 10)); // panasonic 6071 ���O 0.1Nm
  return false;
}

bool PanasonicA6::setMotionProfile(const MotionProfile& p) {
  return setVel(p.vel) && setAcc(p.acc) && setDec(p.dec);
}

bool PanasonicA6::setVel(int vel) {
  if (!ethercat_) {
    return false;
  }
  uint32_t vel_cmdps = rpm_to_cmdps(vel);
  int ok = 1;
  ok &= ethercat_->write_sdo_state(slave_, 0x6081, 0x00, &vel_cmdps, sizeof(vel_cmdps));
  printf("Set 6081 vel=%d rpm -> %u cmd/s\n", vel, vel_cmdps);
  if (!ok) {
    return false;
  }
  return true;
}

bool PanasonicA6::setAcc(int acc) {
  if (!ethercat_) {
    return false;
  }
  uint32_t acc_cmd = rpmps_to_cmdps2(acc);

  int ok = 1;
  ok &= ethercat_->write_sdo_state(slave_, 0x6083, 0x00, &acc_cmd, sizeof(acc_cmd));

  printf("Set acc = %d rpm/s -> %u cmd/s^2\n",
       acc, acc_cmd);
  if (!ok) {
    return false;
  }
  return true;
}

bool PanasonicA6::setDec(int dec) {
  if (!ethercat_) {
    return false;
  }
  uint32_t dec_cmd = rpmps_to_cmdps2(dec);

  int ok = 1;
  ok &= ethercat_->write_sdo_state(slave_, 0x6084, 0x00, &dec_cmd, sizeof(dec_cmd));

  printf("Set dec = %d rpm/s -> %u cmd/s^2\n",
       dec, dec_cmd);
  if (!ok) {
    return false;
  }
  return true;
}

void PanasonicA6::dump_pdo(uint16 slave) {
  if (!ethercat_) {
    return;
  }
  uint8 *out = ethercat_->get_slave_outputs(slave);
  uint8 *in = ethercat_->get_slave_inputs(slave);
  if (!out || !in) {
    return;
  }
  int obytes = ethercat_->get_slave_obytes(slave);
  int ibytes = ethercat_->get_slave_ibytes(slave);

  printf("==== PDO DUMP slave %d ====\n", slave);

  printf("Obytes=%d: ", obytes);
  for (int i = 0; i < obytes; i++) {
    printf("%02X ", out[i]);
  }
  printf("\n");

  printf("Ibytes=%d: ", ibytes);
  for (int i = 0; i < ibytes; i++) {
    printf("%02X ", in[i]);
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
  if (!ethercat_) {
    return false;
  }
  uint8 *out = ethercat_->get_slave_outputs(slave); // 25 bytes
  uint8 *in = ethercat_->get_slave_inputs(slave); // 25 bytes
  if (!out || !in) {
    return false;
  }

  // ? mapping4 �U�A�q�` outputs[0..1] = controlword
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
    uint16 sw = a6_get_u16(in, 2); // ? statusword ���O offset=2 �]�A�� dump �O�ҡ^
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

    a6_set_u16(out, 0, cw); // ? �令 offset=0 !!!

    usleep(10000);

    if (loop++ % 50 == 0) {
      printf("[servoOn] sw=0x%04X st=%d cw=0x%04X modeDisp=%d\n",
             sw, st, cw, in[4]);
      dump_pdo(slave);
    }
  }
}

bool PanasonicA6::servo_on() {
  printf("�}�l Servo ON...\n");
  if (!servoOnPDO_mapping4(slave_)) {
    notify_fatal();
    return false;
  }
  printf("Servo ON ����\n");
  return true;
}

bool PanasonicA6::servoOffPDO_mapping4(uint16 slave) {
  if (!ethercat_) {
    return false;
  }
  uint8 *out = ethercat_->get_slave_outputs(slave);
  uint8 *in = ethercat_->get_slave_inputs(slave);
  if (!out || !in) {
    return false;
  }

  int loop = 0;

  while (1) {
    uint16 sw = a6_get_u16(in, 2); // statusword offset=2
    PDSState st = getPDS(sw);

    // �ؼСG�^�� Switch Disabled
    if (st == SWITCH_DISABLED) {
      printf("[servoOff] SWITCH_DISABLED sw=0x%04X\n", sw);
      return true;
    }

    uint16 cw = 0x0000;

    switch (st) {
      case OP_ENABLED:
        cw = 0x0007; // disable operation �� �^�� switched on
        break;

      case SWITCHED_ON:
        cw = 0x0006; // shutdown �� �^�� ready to switch on
        break;

      case READY_SWITCH:
        cw = 0x0000; // disable voltage �� �^�� switch disabled
        break;

      case FAULT:
        // fault ���A�U servo off �S�N�q�A�� fault reset
        cw = 0x0080;
        break;

      default:
        cw = 0x0000;
        break;
    }

    a6_set_u16(out, 0, cw); // ? controlword offset=0
    usleep(10000); // 10ms

    if (loop++ % 50 == 0) {
      uint16 err = a6_get_u16(in, 0); // 603F error code
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

  if (!ethercat_) {
    return 0;
  }
  uint8 *in = ethercat_->get_slave_inputs(slave_);
  if (!in) {
    return 0;
  }

  // 603F:00 (TxPDO offset 0)
  uint16_t code = a6_get_u16(in, 0);       // e.g. 0xFF50
  int main_code = code & 0x00FF;        // 80

  return main_code;
}

double PanasonicA6::get_current_position() {
  if (!ethercat_) {
    return 0.0;
  }
  uint8 *in = ethercat_->get_slave_inputs(slave_);   // pulse
  if (!in) {
    return 0.0;
  }
  int32 pulse = a6_get_i32(in, 5); // 6064:00 Position actual value
  double deg = static_cast<double>(pulse) * 360.0 / kCountsPerRev;
  return deg;
}

double PanasonicA6::get_current_velocity() {
  if (!ethercat_) {
    return 0.0;
  }
  const uint8 *in = ethercat_->get_slave_inputs(slave_); // cmd/s
  if (!in) {
    return 0.0;
  }
  int32 cmdps = a6_get_i32(in, 9); // 606C:00 Velocity actual value
  return cmdps_to_rpm(cmdps);
}

double PanasonicA6::get_current_torque() {
  if (!ethercat_) {
    return 0.0;
  }
  const uint8 *in = ethercat_->get_slave_inputs(slave_);  // 0.1% rated torque
  if (!in) {
    return 0.0;
  }
  int16 raw = a6_get_i16(in, 13); // 6077:00 Torque actual value
  return static_cast<double>(raw) / 10.0;
}
MotorModes PanasonicA6::get_mode() { return mode_; }
