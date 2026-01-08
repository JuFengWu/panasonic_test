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

bool PanasonicA6B::set_target_position(float target)
{
  (void)target;
  return false;
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
