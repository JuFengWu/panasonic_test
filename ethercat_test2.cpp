#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include "soem/soem.h"
#include <cstring>

#define SLAVE_ID 1   // A6B 在 SOEM 裡的從站號（通常是 1）

/* ====== 新版 SOEM 要有的全域 context ====== */
ecx_contextt ecx_context;


/* ================== SDO 包裝函數 (新版 SOEM - ecx_) ================== */

bool sdo_write_u8(uint16 slave, uint16 index, uint8 sub, uint8 val)
{
    int wkc = ecx_SDOwrite(&ecx_context, slave, index, sub, FALSE, sizeof(val), &val, EC_TIMEOUTSAFE);
    return (wkc > 0);
}

bool sdo_write_u16(uint16 slave, uint16 index, uint8 sub, uint16 val)
{
    int wkc = ecx_SDOwrite(&ecx_context, slave, index, sub, FALSE, sizeof(val), &val, EC_TIMEOUTSAFE);
    return (wkc > 0);
}

bool sdo_write_i32(uint16 slave, uint16 index, uint8 sub, int32 val)
{
    int wkc = ecx_SDOwrite(&ecx_context, slave, index, sub, FALSE, sizeof(val), &val, EC_TIMEOUTSAFE);
    return (wkc > 0);
}

bool sdo_read_u16(uint16 slave, uint16 index, uint8 sub, uint16 *out)
{
    uint16 val = 0;
    int size = sizeof(val);
    int wkc = ecx_SDOread(&ecx_context, slave, index, sub, FALSE, &size, &val, EC_TIMEOUTSAFE);
    if (wkc <= 0) return false;
    *out = val;
    return true;
}


/* ================== 單位換算：角度 → 指令值 ================== */
int32 deg_to_command(double deg)
{
    return (int32)deg;
}


/* ================== A6 馬達控制函數 ================== */

// 設定 Profile Position Mode (6060h = 1)
bool set_pp_mode(uint16 slave)
{
    uint8 mode_pp = 1;
    return sdo_write_u8(slave, 0x6060, 0x00, mode_pp);
}

// CiA402 啟動流程
bool enable_drive(uint16 slave)
{
    if (!sdo_write_u16(slave, 0x6040, 0x00, 0x0006)) return false; // shutdown
    if (!sdo_write_u16(slave, 0x6040, 0x00, 0x0007)) return false; // switch on
    if (!sdo_write_u16(slave, 0x6040, 0x00, 0x000F)) return false; // enable operation
    return true;
}

// Profile Position 絕對位置
bool move_absolute_pp(uint16 slave, double target_deg)
{
    int32 target_cmd = deg_to_command(target_deg);

    // 寫入 Target Position
    if (!sdo_write_i32(slave, 0x607A, 0x00, target_cmd))
        return false;

    // 讀 Controlword
    uint16 cw;
    if (!sdo_read_u16(slave, 0x6040, 0x00, &cw))
        return false;

    cw &= ~(1 << 6);   // bit6 = 0 → 絕對位置
    cw &= ~(1 << 4);   // bit4 = 0
    if (!sdo_write_u16(slave, 0x6040, 0x00, cw)) return false;

    cw |= (1 << 4);    // bit4 = 1 → set-point
    if (!sdo_write_u16(slave, 0x6040, 0x00, cw)) return false;

    // 等待 Target Reached (statusword bit10)
    while (1)
    {
        uint16 sw;
        if (!sdo_read_u16(slave, 0x6041, 0x00, &sw))
            return false;

        if (sw & (1 << 10)) break;

        osal_usleep(10000);
    }

    return true;
}


/* ================== main (新版 SOEM ecx_ API) ================== */
// ===== 必須的 SOEM 全域變數 =====
ec_slavet   ec_slave[EC_MAXSLAVE];
uint8       esibuf[EC_MAXBUF];
uint32      esimap[EC_MAXBUF];
uint8       eep_SM[EC_MAXEEPBUF];
uint8       eep_FP[EC_MAXEEPBUF];

ecx_portt       ecx_port;
ecx_redportt    ecx_redport;

int main(int argc, char *argv[])
{
    const char* ifname = "eth0";

    printf("使用介面卡: %s\n", ifname);

    /* 初始化 context（新版 SOEM 只需要這兩個） */
    memset(&ecx_context, 0, sizeof(ecx_context));
    memset(&ecx_port, 0, sizeof(ecx_portt));

    ecx_context.port       = &ecx_port;
    ecx_context.slavelist  = ec_slave;   // 注意：ec_slave 是 SOEM 的 global array

    /* 開始初始化 EtherCAT */
    if (!ecx_init(&ecx_context, ifname))
    {
        printf("ecx_init 失敗\n");
        return -1;
    }

    printf("ecx_init OK\n");

    if (ecx_config_init(&ecx_context, FALSE) <= 0)
    {
        printf("找不到 EtherCAT 從站\n");
        ecx_close(&ecx_context);
        return -1;
    }

    printf("%d slaves found.\n", ecx_context.slavecount);

    /* 設定 PDO 與 Distributed Clock */
    ecx_config_map(&ecx_context, NULL);
    ecx_configdc(&ecx_context);

    /* SAFE_OP */
    ecx_context.slavelist[0].state = EC_STATE_SAFE_OP;
    ecx_writestate(&ecx_context, 0);

    ecx_send_processdata(&ecx_context);
    ecx_receive_processdata(&ecx_context, EC_TIMEOUTRET);

    /* OP */
    ecx_context.slavelist[0].state = EC_STATE_OPERATIONAL;
    ecx_writestate(&ecx_context, 0);

    int chk = 40;
    do
    {
        ecx_send_processdata(&ecx_context);
        ecx_receive_processdata(&ecx_context, EC_TIMEOUTRET);
        ecx_statecheck(&ecx_context, 0, EC_STATE_OPERATIONAL, 50000);
    }
    while (chk-- && ecx_context.slavelist[0].state != EC_STATE_OPERATIONAL);

    if (ecx_context.slavelist[0].state != EC_STATE_OPERATIONAL)
    {
        printf("沒能成功進入 OP 狀態\n");
        ecx_close(&ecx_context);
        return -1;
    }

    printf("所有從站已進入 OP 狀態\n");

    /* ===== 控制 A6 馬達 ===== */
    if (!set_pp_mode(SLAVE_ID))
    {
        printf("設定 PP 模式失敗\n");
        ecx_close(&ecx_context);
        return -1;
    }

    if (!enable_drive(SLAVE_ID))
    {
        printf("使能伺服失敗\n");
        ecx_close(&ecx_context);
        return -1;
    }

    if (!move_absolute_pp(SLAVE_ID, 30.0))
    {
        printf("移動到 30 度失敗\n");
        ecx_close(&ecx_context);
        return -1;
    }

    printf("已完成從 0° 移動到 30°\n");

    ecx_close(&ecx_context);
    return 0;
}