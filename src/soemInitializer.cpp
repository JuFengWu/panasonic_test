#include "soemInitializer.hpp"
#include "utilities.hpp"
bool SoemInitializer::setup_minasa6b_pdo_mapping4(uint16 slave)
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
void SoemInitializer::print_state(){
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
bool SoemInitializer::set_profile_motion_params(uint16 slave)
{
    // 這些單位是 "internal position unit / s" 或 pulse/s
    // 先用大一點讓你肉眼看得出來動
    uint32 vel = 0x16000000;   // 你原本程式用過的值
    uint32 acc = 0x80000000;
    uint32 dec = 0x80000000;

    bool ok = true;
    ok &= sdo_write_u32(slave, 0x6081, 0x00, vel);
    ok &= sdo_write_u32(slave, 0x6083, 0x00, acc);
    ok &= sdo_write_u32(slave, 0x6084, 0x00, dec);

    printf("Set 6081 vel=%u 6083 acc=%u 6084 dec=%u (%s)\n",
           vel, acc, dec, ok ? "OK" : "FAIL");
    return ok;
}
bool SoemInitializer::motor_initial_connect(const char* ifname, int motor_count, MotorModes mode)
{
  (void)motor_count;
  (void)mode;
  opened_ = true;
  initialized_ = true;

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
  if(ec_slavecount!=motor_count){
      printf("從站數量與預期不符\n");
      ec_close();
      return -1;
  }
  ec_readstate();
  print_state();


  ec_slave[0].state = EC_STATE_PRE_OP;
  ec_writestate(0);

  // 等待真正進 PRE-OP
  ec_statecheck(0, EC_STATE_PRE_OP, EC_TIMEOUTSTATE * 4);
  print_state();

  for (int i = 1; i <= ec_slavecount; i++) {
      setup_minasa6b_pdo_mapping4(i); 
  }
  /// ====== 2. 自動 PDO mapping ======
  ec_config_map(&ioMap);

  //setInterpolationTimePeriod(SLAVE_ID, 4000);

  // ====== 3. 設定 DC ======
  ec_configdc();

  print_state();

  // ====== 4. 主站要求 slave 進 SAFE_OP ======
  ec_statecheck(0, EC_STATE_SAFE_OP, EC_TIMEOUTSTATE);

  if(mode==PP_Mode){
    // ✅ 在 SAFEOP 做一次 SDO 設定 profile motion 參數（最穩）
    for (int i = 1; i <= ec_slavecount; i++) {
        set_profile_motion_params(i);
    }
  }
  
  print_state();

  // ====== 5. 交換 PDO（至少 1 次）======
  ec_send_processdata();
  ec_receive_processdata(EC_TIMEOUTRET);


  return true;
}

bool SoemInitializer::run_async()
{
  if (!opened_) return false;
  running_ = true;
  return true;
}

void SoemInitializer::stop() { running_ = false; }

void SoemInitializer::close()
{
  running_ = false;
  opened_ = false;
}
