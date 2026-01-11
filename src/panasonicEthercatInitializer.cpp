#include "panasonicEthercatInitializer.hpp"
#include "motionSystem.hpp"

#include <chrono>
#include <functional>


bool PanasonicEthercatInitializer::setup_minasa6b_pdo_mapping4(uint16 slave)
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
void PanasonicEthercatInitializer::print_state(){
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
bool PanasonicEthercatInitializer::set_profile_motion_params(uint16 slave) // TODO: let user set these values
{
    // 這些單位是 "internal position unit / s" 或 pulse/s
    // 先用大一點讓你肉眼看得出來動
    uint32 vel = 0x16000000;   // 你原本程式用過的值
    uint32 acc = 0x80000000;
    uint32 dec = 0x80000000;

    bool ok = true;
    ok &= sdo_write_u32(slave, 0x6081, 0x00, vel); //Profile velocity
    ok &= sdo_write_u32(slave, 0x6083, 0x00, acc); //Profile acceleration
    ok &= sdo_write_u32(slave, 0x6084, 0x00, dec); //Profile deceleration

    printf("Set 6081 vel=%u 6083 acc=%u 6084 dec=%u (%s)\n",
           vel, acc, dec, ok ? "OK" : "FAIL");
    return ok;
}
bool PanasonicEthercatInitializer::motor_initial_connect(const char* ifname, int motor_count, MotorModes mode)
{
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
void PanasonicEthercatInitializer::init_motion_params_pdo(uint16 slave, MotorModes mode)
{
    uint8 *out = (uint8*)ec_slave[slave].outputs;

    // mode = PP
    if (mode == PP_Mode){
      out[2] = 1;  // PP mode
    } else if (mode == CSV_Mode) {
      out[2] = 9;  // CSV mode
    } else if (mode == CST_Mode) {
      out[2] = 10;  // CST mode
    } else if (mode == CSP_Mode){
      out[2] = 8;  // CSP mode
    } else{
      out[2] = 1;  // PP mode
    }
    
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

bool PanasonicEthercatInitializer::run_async(CyclicSession& session, AllMotors& motors)
{
  if (!opened_) return false;
  if (running_) return true;
  if (worker_.joinable()) {
    worker_.join();
  }
  running_ = true;
  motors_ = &motors;

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

  worker_ = std::thread(&PanasonicEthercatInitializer::run_loop, this, std::ref(session), std::ref(motors));
  osal_usleep(1000000);  // 10ms

  if (ec_slave[0].state != EC_STATE_OPERATIONAL)
  {
      printf("沒能成功進入 OP 狀態\n");
      ec_close();
      return false;
  }

  printf("所有從站已進入 OP 狀態\n");

  for (int i = 1; i <= ec_slavecount; i++) {
    auto motorMode = motors.motor(i).get_mode(); // 確保 mode 正確
    init_motion_params_pdo(i, motorMode);
  }

  return true;
}
bool PanasonicEthercatInitializer::shutdown_ecat(int pdo_cycle_us)
{
    bool ok = true;

    printf("\n========== shutdown_ecat() ==========\n");

    // 1) Servo OFF（若失敗也繼續往下做 SAFE_OP 避免 watchdog）
    printf("[shutdown] Servo OFF...\n");

    if (motors_) {
      for (int i = 1; i <= ec_slavecount; i++) {
        motors_->motor(i).servo_off();
      }
    }
    printf("[shutdown] Servo OFF done\n");
    

    // 2) Servo OFF 後保持 PDO 交換 300ms
    printf("[shutdown] Keep PDO exchange 300ms...\n");
    usleep(300000);

    // 3) 退回 SAFE_OP（最重要，避免 watchdog）
    printf("[shutdown] Switch master to SAFE_OP...\n");
    ec_slave[0].state = EC_STATE_SAFE_OP;
    ec_writestate(0);

    // 等待 master/slaves 真正進 SAFE_OP
    if (ec_statecheck(0, EC_STATE_SAFE_OP, EC_TIMEOUTSTATE) != EC_STATE_SAFE_OP)
    {
        printf("[shutdown] WARNING: Not all slaves reached SAFE_OP\n");
        ok = false;
    }
    else
    {
        printf("[shutdown] SAFE_OP reached\n");
    }

    // 4) 再送 200ms PDO（讓 transition 穩定）
    printf("[shutdown] Keep PDO exchange 200ms...\n");
    usleep(200000);

    // 5) 停 PDO thread
    printf("[shutdown] Stop PDO thread...\n");
    running_ = false;

    if (worker_.joinable()) {
      worker_.join();
      printf("[shutdown] PDO thread joined\n");
    }

    // 6) close EtherCAT
    printf("[shutdown] ec_close()\n");
    ec_close();

    printf("[shutdown] Done. ok=%d\n", ok);
    printf("=====================================\n\n");

    return ok;
}
void PanasonicEthercatInitializer::motor_stop()
{
  running_ = false;
  if (worker_.joinable()) {
    worker_.join();
  }
}

void PanasonicEthercatInitializer::motor_close()
{
  //motor_stop();
  
  shutdown_ecat();
  running_ = false;
  motors_ = nullptr;
  if (opened_) {
    ec_close();
    opened_ = false;
  }
}

void PanasonicEthercatInitializer::run_loop(CyclicSession& session, AllMotors& motors)
{
  /*using clock = std::chrono::steady_clock;
  auto next = clock::now();
  while (running_) {
    next += std::chrono::milliseconds(4);
    ec_send_processdata();
    ec_receive_processdata(EC_TIMEOUTRET);
    bool break_loop = false;
    session.run(motors, break_loop);
    if (break_loop) {
      running_ = false;
      break;
    }
    std::this_thread::sleep_until(next);
  }*/


  const int cycle_ns = 4 * 1000 * 1000; // 4ms = 4,000,000 ns

    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);

    while (running_)
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
        bool break_loop = false;
        session.run(motors, break_loop);
        if (break_loop) {
          running_ = false;
          break;
        }
        // sleep 到下一個 tick（比 usleep 穩定）
        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &ts, NULL);
    }
}
