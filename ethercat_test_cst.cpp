#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include "utilities.hpp"
#include <unistd.h>
#define SLAVE_ID 1   // A6B 在 SOEM 裡的從站號（通常是 1）
#define COUNTS_PER_REV  (double)0x800000  // 8388608

/* ================== 單位換算：角度 → 指令值 ================== */

// 假設：電子齒輪設定為 1，且「1 指令單位 = 1 度」。
int32 deg_to_command(double deg)
{
    return (int32)(deg / 360.0 * COUNTS_PER_REV);
}

/* ================== A6 馬達控制函數 ================== */

// 設定 Profile Position Mode (6060h = 1)
bool set_pp_mode(uint16 slave)
{
    uint8 mode_pp = 1;  // Profile Position Mode
    if (!sdo_write_u8(slave, 0x6060, 0x00, mode_pp))
    {
        printf("set_pp_mode: 寫 6060h 失敗\n");
        return false;
    }
    return true;
}

void dump_pdo(uint16 slave)
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
typedef enum {
    SWITCH_DISABLED,
    READY_SWITCH,
    SWITCHED_ON,
    OP_ENABLED,
    FAULT,
    UNKNOWN
} PDSState;

PDSState getPDS(uint16 sw)
{
    if ((sw & 0x004F) == 0x0040) return SWITCH_DISABLED;
    if ((sw & 0x006F) == 0x0021) return READY_SWITCH;
    if ((sw & 0x006F) == 0x0023) return SWITCHED_ON;
    if ((sw & 0x006F) == 0x0027) return OP_ENABLED;
    if ((sw & 0x004F) == 0x0008) return FAULT;
    return UNKNOWN;
}

bool servoOnPDO_mapping4(uint16 slave)
{
    uint8 *out = (uint8*)ec_slave[slave].outputs; // 25 bytes
    uint8 *in  = (uint8*)ec_slave[slave].inputs;  // 25 bytes

    // ✅ mapping4 下，通常 outputs[0..1] = controlword
    // ✅ outputs[2] = mode (6060)
    //out[2] = 1;  // PP mode
    //out[2] = 8; // CSP
    //out[2] = 9; // CSV
    out[2] = 10; // CST

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

void init_motion_params_pdo(uint16 slave)
{
    uint8 *out = (uint8*)ec_slave[slave].outputs;

    // mode = PP
    out[2] = 10;

    // 6071 target torque = 0 (PP 不用)
    set_u16(out, 3, 0);

    // 6072 max torque (unit: 0.1% or 1% 視驅動器)
    // 先給 1000 (常見代表 100%)
    set_u16(out, 5, 1000);

    // 6080 max motor speed (你原本程式用 0x16000000)
    set_i32(out, 11, 0x16000000);

    // 60B8 touch probe function = 0
    set_u16(out, 15, 0);

    // 60FF target velocity = 0 (PP 不用)
    set_i32(out, 17, 0);
}
bool servoOffPDO_mapping4(uint16 slave)
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
        }
    }
}

void print_state(){
    ec_readstate();
    printf("++state++\n");
    for (int i = 1; i <= ec_slavecount; i++) {  // 正確：從 1 開始
        printf("Slave %d state=0x%02X, AL=0x%04X\n",
            i,
            ec_slave[i].state,
            ec_slave[i].ALstatuscode
        );
    }
    printf("=====\n");
}

bool setTrqueForEmergencyStop(uint16 slave, double val)
{
    int16 i16val = (int16)val;
    return sdo_write_i16(slave, 0x3511, 0x00, i16val);
}

bool setOverLoadLevel(uint16 slave, double val)
{
    int16 i16val = (int16)val;
    return sdo_write_i16(slave, 0x3512, 0x00, i16val);
}

bool setOverSpeedLevel(uint16 slave, double val)
{
    int16 i16val = (int16)val;
    return sdo_write_i16(slave, 0x3513, 0x00, i16val);
}

bool setMotorWorkingRange(uint16 slave, double val)
{
    // 3514h: unit 0.1 revolute
    int16 i16val = (int16)(val * 10);
    return sdo_write_i16(slave, 0x3514, 0x00, i16val);
}
bool setProfileVelocity(uint16 slave, uint32 val)
{
    return sdo_write_u32(slave, 0x6081, 0x00, val);
}

bool setProfileAcceleration(uint16 slave, uint32 val)
{
    return sdo_write_u32(slave, 0x6083, 0x00, val);
}

bool setProfileDeceleration(uint16 slave, uint32 val)
{
    return sdo_write_u32(slave, 0x6084, 0x00, val);
}

bool setInterpolationTimePeriod(uint16 slave, int us)
{
    uint32 u32val;
    uint8 u8val;

    switch (us)
    {
    case 250:  u32val = 250000;   u8val = 25; break;
    case 500:  u32val = 500000;   u8val = 5;  break;
    case 1000: u32val = 1000000;  u8val = 1;  break;
    case 2000: u32val = 2000000;  u8val = 2;  break;
    case 4000: u32val = 4000000;  u8val = 4;  break;
    default:
        printf("setInterpolationTimePeriod(%d) must be 250,500,1000,2000,4000\n", us);
        return false;
    }

    int ok = 1;
    ok &= sdo_write_u32(slave, 0x1C32, 0x02, u32val);
    ok &= sdo_write_u8(slave, 0x60C2, 0x01, u8val);

    uint32 r32; uint8 r8;
    sdo_read_u32(slave, 0x1C32, 0x02, &r32);
    sdo_read_u8(slave, 0x60C2, 0x01, &r8);

    printf("Set interpolation time period %d us\n", us);
    printf("1C32:02 cycle time = %u ns\n", r32);
    printf("60C2:01 interpolation time period = %u\n", r8);

    return ok;
}

char ioMap[4096];

static volatile int run_pdo = 1;
static pthread_t pdo_thread;

void *pdo_loop_thread(void *arg)
{
    const int cycle_ns = 4 * 1000 * 1000; // 4ms = 4,000,000 ns

    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);

    while (run_pdo)
    {
        // 下一個週期時間點
        ts.tv_nsec += cycle_ns;
        while (ts.tv_nsec >= 1000000000)
        {
            ts.tv_nsec -= 1000000000;
            ts.tv_sec += 1;
        }

        ec_send_processdata();
        ec_receive_processdata(EC_TIMEOUTRET);

        // sleep 到下一個 tick（比 usleep 穩定）
        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &ts, NULL);
    }
    return NULL;
}

bool setup_minasa6b_pdo_mapping4(uint16 slave)
{
    int ret = 0, l;
    uint8 num_pdo;
    uint8 num_entries;

    // --- RxPDO mapping object 1603h ---
    l = sizeof(num_entries);
    ret += ec_SDOread(slave, 0x1603, 0x00, FALSE, &l, &num_entries, EC_TIMEOUTRXM);
    printf("RxPDO 1603 current entries = %d\n", num_entries);

    // clear mapping
    num_entries = 0;
    ret += ec_SDOwrite(slave, 0x1603, 0x00, FALSE, sizeof(num_entries), &num_entries, EC_TIMEOUTRXM);

    // 這裡如果你只是要用原本手冊 mapping 4，其實不用改 1603 子項目
    // 你貼的 code 有把 0x1603:09 改成 60B0(位置偏移) 只是額外功能
    // 如果要照貼的 code 做：
    uint32 mapping = 0x60B00020; // 60B0:00 32bit
    ret += ec_SDOwrite(slave, 0x1603, 0x09, FALSE, sizeof(mapping), &mapping, EC_TIMEOUTRXM);

    // set entries back
    num_entries = 9;
    ret += ec_SDOwrite(slave, 0x1603, 0x00, FALSE, sizeof(num_entries), &num_entries, EC_TIMEOUTRXM);

    // --- Assign RxPDO (1C12) ---
    num_pdo = 0;
    ret += ec_SDOwrite(slave, 0x1C12, 0x00, FALSE, sizeof(num_pdo), &num_pdo, EC_TIMEOUTRXM);

    uint16 idx_rxpdo = 0x1603;
    ret += ec_SDOwrite(slave, 0x1C12, 0x01, FALSE, sizeof(idx_rxpdo), &idx_rxpdo, EC_TIMEOUTRXM);

    num_pdo = 1;
    ret += ec_SDOwrite(slave, 0x1C12, 0x00, FALSE, sizeof(num_pdo), &num_pdo, EC_TIMEOUTRXM);

    // --- Assign TxPDO (1C13) ---
    num_pdo = 0;
    ret += ec_SDOwrite(slave, 0x1C13, 0x00, FALSE, sizeof(num_pdo), &num_pdo, EC_TIMEOUTRXM);

    uint16 idx_txpdo = 0x1A03;
    ret += ec_SDOwrite(slave, 0x1C13, 0x01, FALSE, sizeof(idx_txpdo), &idx_txpdo, EC_TIMEOUTRXM);

    num_pdo = 1;
    ret += ec_SDOwrite(slave, 0x1C13, 0x00, FALSE, sizeof(num_pdo), &num_pdo, EC_TIMEOUTRXM);

    printf("setup_minasa6b_pdo_mapping4 ret=%d\n", ret);
    return (ret > 0);
}

void init_cst_mode(uint16 slave)
{
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

static inline void cst_set_target_torque(uint16 slave, int16 tq_cmd)
{
    uint8 *out = (uint8*)ec_slave[slave].outputs;
    set_i16(out, 3, tq_cmd);      // 6071 offset=3
}
void cst_torque_test(uint16 slave, int16 tq_cmd, int cycles)
{
    const int dt_us = 4000;          // 4ms
    const int run_ms = 2000;         // 每次跑 2 秒
    const int steps = run_ms / 4;    // 2s / 4ms = 500 次更新

    for (int c = 0; c < cycles; c++)
    {
        printf("=== CST Cycle %d: +Torque %d for %dms ===\n", c+1, tq_cmd, run_ms);
        for (int i = 0; i < steps; i++)
        {
            cst_set_target_torque(slave, tq_cmd);
            usleep(dt_us);
        }

        // torque = 0
        cst_set_target_torque(slave, 0);
        usleep(200000);

        printf("=== CST Cycle %d: -Torque %d for %dms ===\n", c+1, tq_cmd, run_ms);
        for (int i = 0; i < steps; i++)
        {
            cst_set_target_torque(slave, -tq_cmd);
            usleep(dt_us);
        }

        // torque = 0
        cst_set_target_torque(slave, 0);
        usleep(200000);
    }

    // 最後一定要歸 0 扭矩
    cst_set_target_torque(slave, 0);
}

/* ================== main: 整個流程串起來 ================== */

int main(int argc, char *argv[])
{
    const char *ifname;

    if (argc > 1)
        ifname = argv[1];         // 例如: ./a.out enp2s0
    else
        ifname = "enp3s0";

    printf("使用介面卡: %s\n", ifname);

    if (!ec_init(ifname))
    {
        printf("ec_init 失敗\n");
        return -1;
    }

    printf("ec_init OK\n");

    if (ec_config_init(FALSE) <= 0)
    {
        printf("找不到 EtherCAT 從站\n");
        ec_close();
        return -1;
    }

    printf("%d slaves found.\n", ec_slavecount);
    ec_readstate();
    print_state();


    ec_slave[0].state = EC_STATE_PRE_OP;
    ec_writestate(0);

    // 等待真正進 PRE-OP
    ec_statecheck(0, EC_STATE_PRE_OP, EC_TIMEOUTSTATE * 4);
    print_state();

    setup_minasa6b_pdo_mapping4(SLAVE_ID); 

    /// ====== 2. 自動 PDO mapping ======
    ec_config_map(&ioMap);


    //setInterpolationTimePeriod(SLAVE_ID, 4000);

    // ====== 3. 設定 DC ======
    ec_configdc();

    print_state();

    // ====== 4. 主站要求 slave 進 SAFE_OP ======
    ec_statecheck(0, EC_STATE_SAFE_OP, EC_TIMEOUTSTATE);

    // ✅ 在 SAFEOP 做一次 SDO 設定 profile motion 參數（最穩）

    // ====== 5. 交換 PDO（至少 1 次）======
    ec_send_processdata();
    ec_receive_processdata(EC_TIMEOUTRET);

    // ====== 6. 主站要求 slave 進 OP ======
    ec_slave[0].state = EC_STATE_OPERATIONAL;
    ec_writestate(0);

    // 等待 slave 真正進 OP
    ec_statecheck(0, EC_STATE_OPERATIONAL, EC_TIMEOUTSTATE);
    print_state();
    
    // 等待達到 OP
    int chk = 40;
    do
    {
        ec_send_processdata();
        ec_receive_processdata(EC_TIMEOUTRET);
        ec_statecheck(0, EC_STATE_OPERATIONAL, 50000);
    } while (chk-- && (ec_slave[0].state != EC_STATE_OPERATIONAL));

    // ✅ 啟動 4ms PDO loop thread（避免 80.4 watchdog）
    printf("啟動 4ms PDO loop thread...\n");

    pthread_create(&pdo_thread, NULL, pdo_loop_thread, NULL);

    osal_usleep(1000000);  // 10ms

    if (ec_slave[0].state != EC_STATE_OPERATIONAL)
    {
        printf("沒能成功進入 OP 狀態\n");
        ec_close();
        return -1;
    }

    printf("所有從站已進入 OP 狀態\n");


    // ✅ 設定 interpolation time period 為 4ms（等同參考 MinasClient）
    /*if (!setInterpolationTimePeriod(SLAVE_ID, 4000))
    {
        printf("設定 interpolation time period 失敗\n");
        run_pdo = 0;
        pthread_join(pdo_thread, NULL);
        ec_close();
        return -1;
    }*/


    uint16 sw;
    sdo_read_u16(SLAVE_ID, 0x6041, 0x00, &sw);
    printf("Statusword = 0x%04X\n", sw);


    // ✅ 設定 Profile Position Mode
    if (!set_pp_mode(SLAVE_ID))
    {
        printf("設定 PP 模式失敗\n");
        run_pdo = 0;
        pthread_join(pdo_thread, NULL);
        ec_close();
        return -1;
    }

    init_motion_params_pdo(SLAVE_ID);

    // ✅ Servo ON
    printf("開始 Servo ON...\n");
    if (!servoOnPDO_mapping4(SLAVE_ID))
    {
        printf("使能伺服失敗\n");
        run_pdo = 0;
        pthread_join(pdo_thread, NULL);
        ec_close();
        return -1;
    }
    printf("Servo ON 完成\n");

    init_cst_mode(SLAVE_ID);

    // 3) 扭矩測試：例如 tq_cmd=100（先小一點），跑 5 cycles
    cst_torque_test(SLAVE_ID, 100, 5);


    // ✅ Servo OFF
    printf("開始 Servo OFF...\n");
    if (!servoOffPDO_mapping4(SLAVE_ID))
    {
        printf("Servo OFF 失敗\n");
        run_pdo = 0;
        pthread_join(pdo_thread, NULL);
        ec_close();
        return -1;
    }
    printf("Servo OFF 完成\n");

    

    // ✅ 停掉 PDO thread
    run_pdo = 0;
    pthread_join(pdo_thread, NULL);

    ec_close();
    return 0;
}
