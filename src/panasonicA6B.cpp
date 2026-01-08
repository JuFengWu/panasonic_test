#include "panasonicA6B.hpp"
#include <unistd.h>
Motor::Motor(int slave, MotorModes mode) : slave_(slave), mode_(mode) {}

PanasonicA6B::PanasonicA6B(int slave, MotorModes mode)
    : Motor(slave, mode) {}

bool PanasonicA6B::set_mode(MotorModes mode)
{
  mode_ = mode;
  return true;
}
// 假設：電子齒輪設定為 1，且「1 指令單位 = 1 度」。
int32 PanasonicA6B::deg_to_command(double deg)
{
    return (int32)(deg / 360.0 * kCountsPerRev);
}
bool PanasonicA6B::move_absolute_pp_pdo(uint16 slave, double target_deg)
{
    uint8 *out = (uint8*)ec_slave[slave].outputs;
    uint8 *in  = (uint8*)ec_slave[slave].inputs;

    int32 target_cmd = deg_to_command(target_deg);

    printf("=== Before move ===\n");
    //dump_pdo(slave);

    // 0) 等 bit10 先變 0（確保下一次會重新變 1）
    int t = 0;
    while (t < 1000)
    {
        uint16 sw = get_u16(in, 2);
        if (!(sw & (1 << 10))) break;
        usleep(1000);
        t += 1;
    }

    // 1) 寫 target position (你目前用 offset=7)
    set_i32(out, 7, target_cmd);

    // 2) 控制字準備
    uint16 cw = get_u16(out, 0);
    cw |= 0x000F;          // enable
    cw &= ~(1 << 6);       // absolute

    // 先清 bit4
    cw &= ~(1 << 4);
    set_u16(out, 0, cw);
    usleep(2000);

    // 再 set bit4
    cw |= (1 << 4);
    set_u16(out, 0, cw);

    // 3) 等 bit12 ack = 1
    t = 0;
    while (t < 2000)
    {
        uint16 sw = get_u16(in, 2);
        if (sw & (1 << 12)) break;
        usleep(1000);
        t += 1;
    }

    // 4) 等 bit10 reached = 1 + debug current position
    int timeout_ms = 5000;
    int elapsed = 0;
    int next_print = 0;

    while (elapsed < timeout_ms)
    {
        uint16 sw = get_u16(in, 2);
        int32 act = get_i32(in, 5);   // 先用 5，但我們用 dump 來確認

        if (elapsed >= next_print)
        {
            printf("[move dbg] t=%4dms sw=0x%04X act=%d target=%d diff=%d\n",
                   elapsed, sw, act, target_cmd, target_cmd - act);
            next_print += 100;
        }

        if (sw & (1 << 10))
        {
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
bool PanasonicA6B::set_target_position(float target)
{
  (void)target;
  if (mode_ == PP_Mode){
    return move_absolute_pp_pdo(slave_, target);
  }
}

bool PanasonicA6B::set_target_velocity(float target)
{
  (void)target;
  return false;
}

bool PanasonicA6B::set_target_torque(float target)
{
  (void)target;
  return false;
}

bool PanasonicA6B::setMotionProfile(const MotionProfile& p)
{
  return setVel(p.vel) && setAcc(p.acc) && setDec(p.dec);
}

bool PanasonicA6B::setVel(int vel)
{
  (void)vel;
  return false;
}

bool PanasonicA6B::setAcc(int acc)
{
  (void)acc;
  return false;
}

bool PanasonicA6B::setDec(int dec)
{
  (void)dec;
  return false;
}
void PanasonicA6B::dump_pdo(uint16 slave)
{
  printf("==== PDO DUMP slave %d ====\n", slave);

  printf("Obytes=%d: ", ec_slave[slave].Obytes);
  for(int i=0;i<ec_slave[slave].Obytes;i++)
      printf("%02X ", ((uint8*)ec_slave[slave].outputs)[i]);
  printf("\n");

  printf("Ibytes=%d: ", ec_slave[slave].Ibytes);
  for(int i=0;i<ec_slave[slave].Ibytes;i++)
      printf("%02X ", ((uint8*)ec_slave[slave].inputs)[i]);
  printf("\n");
  printf("==========================\n");
}

PanasonicA6B::PDSState PanasonicA6B::getPDS(uint16 sw)
{
    if ((sw & 0x004F) == 0x0040) return SWITCH_DISABLED;
    if ((sw & 0x006F) == 0x0021) return READY_SWITCH;
    if ((sw & 0x006F) == 0x0023) return SWITCHED_ON;
    if ((sw & 0x006F) == 0x0027) return OP_ENABLED;
    if ((sw & 0x004F) == 0x0008) return FAULT;
    return UNKNOWN_PDS_STATE;
}

bool PanasonicA6B::servoOnPDO_mapping4(uint16 slave)
{
    uint8 *out = (uint8*)ec_slave[slave].outputs; // 25 bytes
    uint8 *in  = (uint8*)ec_slave[slave].inputs;  // 25 bytes

    // ✅ mapping4 下，通常 outputs[0..1] = controlword
    // ✅ outputs[2] = mode (6060)
    out[2] = 1;  // PP mode

    int loop = 0;
    while (1)
    {
        uint16 sw = get_u16(in, 2);  // ✅ statusword 仍是 offset=2 （你的 dump 保證）
        PDSState st = getPDS(sw);

        if (st == OP_ENABLED)
        {
            printf("[servoOn] OPERATION_ENABLED sw=0x%04X\n", sw);
            return true;
        }

        uint16 cw = 0x0006;
        switch (st)
        {
            case SWITCH_DISABLED: cw = 0x0006; break;
            case READY_SWITCH:    cw = 0x0007; break;
            case SWITCHED_ON:     cw = 0x000F; break;
            case FAULT:           cw = 0x0080; break;
            default:              cw = 0x0006; break;
        }

        set_u16(out, 0, cw);  // ✅ 改成 offset=0 !!!

        usleep(10000);

        if (loop++ % 50 == 0)
        {
            printf("[servoOn] sw=0x%04X st=%d cw=0x%04X modeDisp=%d\n",
                   sw, st, cw, in[4]);
            dump_pdo(slave);
        }
    }
}
bool PanasonicA6B::servo_on()
{
  printf("開始 Servo ON...\n");
  if (!servoOnPDO_mapping4(slave_))
  {
    notify_fatal();
    return false;
  }
  printf("Servo ON 完成\n");
  return true;
}
bool PanasonicA6B::servoOffPDO_mapping4(uint16 slave)
{
    uint8 *out = (uint8*)ec_slave[slave].outputs;
    uint8 *in  = (uint8*)ec_slave[slave].inputs;

    int loop = 0;

    while (1)
    {
        uint16 sw = get_u16(in, 2);   // statusword offset=2
        PDSState st = getPDS(sw);

        // ✅ 目標：回到 Switch Disabled
        if (st == SWITCH_DISABLED)
        {
            printf("[servoOff] SWITCH_DISABLED sw=0x%04X\n", sw);
            return true;
        }

        uint16 cw = 0x0000;

        switch (st)
        {
            case OP_ENABLED:
                cw = 0x0007;  // disable operation → 回到 switched on
                break;

            case SWITCHED_ON:
                cw = 0x0006;  // shutdown → 回到 ready to switch on
                break;

            case READY_SWITCH:
                cw = 0x0000;  // disable voltage → 回到 switch disabled
                break;

            case FAULT:
                // fault 狀態下 servo off 沒意義，給 fault reset
                cw = 0x0080;
                break;

            default:
                cw = 0x0000;
                break;
        }

        set_u16(out, 0, cw);  // ✅ controlword offset=0
        usleep(10000);        // 10ms

        if (loop++ % 50 == 0)
        {
            uint16 err = get_u16(in, 0); // 603F error code
            printf("[servoOff] sw=0x%04X st=%d cw=0x%04X err=0x%04X\n",
                   sw, st, cw, err);
        } //TODO?  retun false if too long??
    }
}
bool PanasonicA6B::servo_off()
{
  return servoOffPDO_mapping4(slave_);
}

int PanasonicA6B::get_error_code() { return 0; }

int PanasonicA6B::get_current_position() { return 0; }

int PanasonicA6B::get_current_velocity() { return 0; }

int PanasonicA6B::get_current_torque() { return 0; }

MotorModes PanasonicA6B::get_mode() { return mode_; }
