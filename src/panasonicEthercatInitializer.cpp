#include "panasonicEthercatInitializer.hpp"
#include "motionSystem.hpp"
#include "MuEthercat.hpp"

#include <cstdint>
#include <chrono>
#include <functional>
#include <time.h>
#include <utility>

namespace {
using uint8 = std::uint8_t;
using uint16 = std::uint16_t;
using uint32 = std::uint32_t;
using int16 = std::int16_t;
using int32 = std::int32_t;

inline bool sdo_write_u8(MyEthercat& ethercat, uint16 slave, uint16 index, uint8 sub, uint8 val) {
    return ethercat.write_sdo_state(slave, index, sub, &val, static_cast<int>(sizeof(val)));
}

inline bool sdo_write_u16(MyEthercat& ethercat, uint16 slave, uint16 index, uint8 sub, uint16 val) {
    return ethercat.write_sdo_state(slave, index, sub, &val, static_cast<int>(sizeof(val)));
}

inline bool sdo_write_u32(MyEthercat& ethercat, uint16 slave, uint16 index, uint8 sub, uint32 val) {
    return ethercat.write_sdo_state(slave, index, sub, &val, static_cast<int>(sizeof(val)));
}

inline bool sdo_read_u8(MyEthercat& ethercat, uint16 slave, uint16 index, uint8 sub, uint8* out) {
    return ethercat.read_sdo_state(slave, index, sub, out, static_cast<int>(sizeof(*out)));
}

inline bool sdo_read_u32(MyEthercat& ethercat, uint16 slave, uint16 index, uint8 sub, uint32* out) {
    return ethercat.read_sdo_state(slave, index, sub, out, static_cast<int>(sizeof(*out)));
}

static inline void pe_set_i16(uint8 *p, int off, int16 v) {
  p[off] = static_cast<uint8>(v & 0xFF);
  p[off + 1] = static_cast<uint8>((v >> 8) & 0xFF);
}

static inline int16 pe_get_i16(const uint8 *p, int off) {
  return static_cast<int16>(p[off] | (p[off + 1] << 8));
}

static inline uint16 pe_get_u16(const uint8 *p, int off) {
  return static_cast<uint16>(p[off] | (p[off + 1] << 8));
}

static inline void pe_set_u16(uint8 *p, int off, uint16 v) {
  p[off] = static_cast<uint8>(v & 0xFF);
  p[off + 1] = static_cast<uint8>((v >> 8) & 0xFF);
}

static inline int32 pe_get_i32(const uint8 *p, int off) {
  return static_cast<int32>(p[off] | (p[off + 1] << 8) | (p[off + 2] << 16) |
                 (p[off + 3] << 24));
}

static inline void pe_set_i32(uint8 *p, int off, int32 v) {
  p[off] = static_cast<uint8>(v & 0xFF);
  p[off + 1] = static_cast<uint8>((v >> 8) & 0xFF);
  p[off + 2] = static_cast<uint8>((v >> 16) & 0xFF);
  p[off + 3] = static_cast<uint8>((v >> 24) & 0xFF);
}
} // namespace



PanasonicEthercatInitializer::PanasonicEthercatInitializer()
//    : ethercat_(std::make_shared<SoemEthercat>()) {}
    : ethercat_(std::make_shared<MuEthercat>()) {}


bool PanasonicEthercatInitializer::setup_minasa6b_pdo_mapping4(uint16 slave)
{
    if (!ethercat_) {
        return false;
    }

    int ret = 0;
    uint8 num_pdo = 0;
    uint8 num_entries = 0;

    // --- RxPDO mapping object 1603h ---
    ret += sdo_read_u8(*ethercat_, slave, 0x1603, 0x00, &num_entries) ? 1 : 0;
    printf("RxPDO 1603 current entries = %d\n", num_entries);

    // clear mapping
    num_entries = 0;
    ret += sdo_write_u8(*ethercat_, slave, 0x1603, 0x00, num_entries) ? 1 : 0;

    // 這裡如果你只是要用原本手冊 mapping 4，其實不用改 1603 子項目
    // 你貼的 code 有把 0x1603:09 改成 60B0(位置偏移) 只是額外功能
    // 如果要照貼的 code 做：
    uint32 mapping = 0x60B00020; // 60B0:00 32bit
    ret += sdo_write_u32(*ethercat_, slave, 0x1603, 0x09, mapping) ? 1 : 0;

    // set entries back
    num_entries = 9;
    ret += sdo_write_u8(*ethercat_, slave, 0x1603, 0x00, num_entries) ? 1 : 0;

    // --- Assign RxPDO (1C12) ---
    num_pdo = 0;
    ret += sdo_write_u8(*ethercat_, slave, 0x1C12, 0x00, num_pdo) ? 1 : 0;

    uint16 idx_rxpdo = 0x1603;
    ret += sdo_write_u16(*ethercat_, slave, 0x1C12, 0x01, idx_rxpdo) ? 1 : 0;

    num_pdo = 1;
    ret += sdo_write_u8(*ethercat_, slave, 0x1C12, 0x00, num_pdo) ? 1 : 0;

    // --- Assign TxPDO (1C13) ---
    num_pdo = 0;
    ret += sdo_write_u8(*ethercat_, slave, 0x1C13, 0x00, num_pdo) ? 1 : 0;

    uint16 idx_txpdo = 0x1A03;
    ret += sdo_write_u16(*ethercat_, slave, 0x1C13, 0x01, idx_txpdo) ? 1 : 0;

    num_pdo = 1;
    ret += sdo_write_u8(*ethercat_, slave, 0x1C13, 0x00, num_pdo) ? 1 : 0;

    printf("setup_minasa6b_pdo_mapping4 ret=%d\n", ret);
    return (ret > 0);
}
bool PanasonicEthercatInitializer::setInterpolationTimePeriod(uint16 slave, int us)
{
    if (!ethercat_) {
        return false;
    }

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
    ok &= sdo_write_u32(*ethercat_, slave, 0x1C32, 0x02, u32val);
    ok &= sdo_write_u8(*ethercat_, slave, 0x60C2, 0x01, u8val);

    uint32 r32; uint8 r8;
    sdo_read_u32(*ethercat_, slave, 0x1C32, 0x02, &r32);
    sdo_read_u8(*ethercat_, slave, 0x60C2, 0x01, &r8);

    printf("Set interpolation time period %d us\n", us);
    printf("1C32:02 cycle time = %u ns\n", r32);
    printf("60C2:01 interpolation time period = %u\n", r8);

    return ok;
}
void PanasonicEthercatInitializer::print_state(){
    if (!ethercat_) {
        return;
    }
    ethercat_->read_ehtercat_state();
    printf("++state++\n");
    int count = ethercat_->get_slave_count();
    for (int i = 1; i <= count; i++) {  // 正確：從 1 開始
        printf("Slave %d state=0x%02X, AL=0x%04X\n",
            i,
            ethercat_->get_slave_state(static_cast<uint16>(i)),
            ethercat_->get_slave_al_status(static_cast<uint16>(i))
        );
    }
    printf("=====\n");
}
bool PanasonicEthercatInitializer::set_profile_motion_params(uint16 slave) // TODO: let user set these values
{
    if (!ethercat_) {
        return false;
    }

    // 這些單位是 "internal position unit / s" 或 pulse/s
    // 先用大一點讓你肉眼看得出來動
    uint32 vel = 0x16000000;   // 你原本程式用過的值
    uint32 acc = 0x80000000;
    uint32 dec = 0x80000000;

    bool ok = true;
    ok &= sdo_write_u32(*ethercat_, slave, 0x6081, 0x00, vel); //Profile velocity
    ok &= sdo_write_u32(*ethercat_, slave, 0x6083, 0x00, acc); //Profile acceleration
    ok &= sdo_write_u32(*ethercat_, slave, 0x6084, 0x00, dec); //Profile deceleration

    printf("Set 6081 vel=%u 6083 acc=%u 6084 dec=%u (%s)\n",
           vel, acc, dec, ok ? "OK" : "FAIL");
    return ok;
}
bool PanasonicEthercatInitializer::get_slave_count(const char* ifname, int& count)
{
  if (!ethercat_) {
    count = 0;
    return false;
  }
  if (!ethercat_->init(ifname)){
    printf("ec_init 失敗\n");
    return false;
  }

  printf("ec_init OK\n");

  if (!ethercat_->scan_slaves()){
    printf("找不到 EtherCAT 從站\n");
    ethercat_->close();
    return false;
  }
  count = ethercat_->get_slave_count();
  printf("%d slaves found.\n", count);
  return true;
}
bool PanasonicEthercatInitializer::motor_initial_connect(const char* ifname, int motor_count,int cyclePeriod, MotorModes mode)
{
  set_cycle_period_ms(cyclePeriod);
  opened_ = true;
  initialized_ = true;

  if (!ethercat_) {
      return false;
  }

  {
      auto mu = std::dynamic_pointer_cast<MuEthercat>(ethercat_);
      if (mu) {
          const std::uint32_t cycle_ns = static_cast<std::uint32_t>(cyclePeriod) * 1000000u;
          mu->set_dc_config(cycle_ns, 0, 0, 0);
      }
  }

  if (!ethercat_->init(ifname))
  {
      printf("ec_init 失敗\n");
      return false;
  }

  printf("ec_init OK\n");

  if (!ethercat_->scan_slaves())
  {
      printf("找不到 EtherCAT 從站\n");
      ethercat_->close();
      return false;
  }

  int slavecount = ethercat_->get_slave_count();
  printf("%d slaves found.\n", slavecount);
  if(slavecount!=motor_count){
      printf("從站數量與預期不符\n");
      ethercat_->close();
      return false;
  }
  ethercat_->read_ehtercat_state();
  print_state();

  ethercat_->set_slave_state(0, static_cast<uint16>(EthercatState::PreOp));

  // 等待真正進 PRE-OP
  ethercat_->state_check(0, static_cast<uint16>(EthercatState::PreOp), ethercat_->timeout_state() * 4);
  print_state();

  for (int i = 1; i <= slavecount; i++) {
      setup_minasa6b_pdo_mapping4(i); 
  }
  /// ====== 2. 自動 PDO mapping ======
  if (ethercat_->config_pdo_mapping(&ioMap) <= 0) {
      printf("config_pdo_mapping failed\n");
      return false;
  }

  // ====== 3. 設定 DC ======
  if (!ethercat_->setting_dc()) {
      printf("setting_dc failed\n");
      return false;
  }

  print_state();

  // ====== 4. 主站要求 slave 進 SAFE_OP ======
  ethercat_->state_check(0, static_cast<uint16>(EthercatState::SafeOp), ethercat_->timeout_state());
  
  for (int i = 1; i <= slavecount; i++) {
      setInterpolationTimePeriod(i, cyclePeriod*1000);// here!!
  }
  
  if(mode==PP_Mode){
    // ✅ 在 SAFEOP 做一次 SDO 設定 profile motion 參數（最穩）
    for (int i = 1; i <= slavecount; i++) {
        set_profile_motion_params(i);
    }
  }
  
  print_state();

  // ====== 5. 交換 PDO（至少 1 次）======
  ethercat_->set_pdo_data();
  ethercat_->get_pdo_data();

  return true;
}
void PanasonicEthercatInitializer::init_motion_params_pdo(uint16 slave, MotorModes mode)
{
    if (!ethercat_) {
        return;
    }
    uint8 *out = ethercat_->get_slave_outputs(slave);
    if (!out) {
        return;
    }

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
    pe_set_u16(out, 3, static_cast<uint16>(0));

    // 6072 max torque (unit: 0.1% or 1% 視驅動器)
    // 先給 1000 (常見代表 100%)
    pe_set_u16(out, 5, static_cast<uint16>(1000));

    // 6080 max motor speed (你原本程式用 0x16000000)
    pe_set_i32(out, 11, 0x16000000);

    // 60B8 touch probe function = 0
    pe_set_u16(out, 15, static_cast<uint16>(0));

    // 60FF target velocity = 0 (PP 不用)
    pe_set_i32(out, 17, 0);
}

bool PanasonicEthercatInitializer::start_async(CyclicSession& session, AllMotors& motors, bool call_session)
{
  if (!opened_) return false;
  if (!ethercat_) return false;
  if (running_) return true;
  if (worker_.joinable()) {
    worker_.join();
  }
  set_call_session_enabled(call_session);
  reset_shutdown_notification();
  running_ = true;
  motors_ = &motors;

  // ====== 6. Set slaves to OP ======
  ethercat_->set_slave_state(0, static_cast<uint16>(EthercatState::Operational));

  // Wait for OP state
  ethercat_->state_check(0, static_cast<uint16>(EthercatState::Operational), ethercat_->timeout_state());
  print_state();
  
  int chk = 40;
  const uint16 op_state = static_cast<uint16>(EthercatState::Operational);
  do
  {
      ethercat_->set_pdo_data();
      ethercat_->get_pdo_data();
      ethercat_->state_check(0, op_state, 50000);
  } while (chk-- && (ethercat_->get_slave_state(0) != op_state));

  // Start PDO loop thread
  printf("Start PDO loop thread...\n");

  worker_ = std::thread(&PanasonicEthercatInitializer::run_loop_impl, this, std::ref(session), std::ref(motors));
  usleep(1000000);  // 10ms

  if (ethercat_->get_slave_state(0) != op_state)
  {
      printf("Failed to reach OP state.\n");
      ethercat_->close();
      return false;
  }

  printf("Reached OP state.\n");

  int count = ethercat_->get_slave_count();
  for (int i = 1; i <= count; i++) {
    auto motorMode = motors.motor(i).get_mode();
    init_motion_params_pdo(static_cast<uint16>(i), motorMode);
  }

  return true;
}

bool PanasonicEthercatInitializer::initializer_run_async(CyclicSession& session, AllMotors& motors)
{
  return start_async(session, motors, true);
}

bool PanasonicEthercatInitializer::initializer_run_async_io_only(CyclicSession& session, AllMotors& motors)
{
  return start_async(session, motors, false);
}
std::thread& PanasonicEthercatInitializer::worker_thread() { return worker_; }

std::atomic<bool>& PanasonicEthercatInitializer::running_flag() { return running_; }

bool PanasonicEthercatInitializer::shutdown_ecat()
{
    if (!ethercat_) {
        return false;
    }

    bool ok = true;

    printf("\n========== shutdown_ecat() ==========\n");

    // 1) Servo OFF（若失敗也繼續往下做 SAFE_OP 避免 watchdog）
    printf("[shutdown] Servo OFF...\n");

    if (motors_) {
      int count = ethercat_->get_slave_count();
      for (int i = 1; i <= count; i++) {
        motors_->motor(i).servo_off();
      }
    }
    printf("[shutdown] Servo OFF done\n");
    

    // 2) Servo OFF 後保持 PDO 交換 300ms
    printf("[shutdown] Keep PDO exchange 300ms...\n");
    usleep(300000);

    // 3) 退回 SAFE_OP（最重要，避免 watchdog）
    printf("[shutdown] Switch master to SAFE_OP...\n");
    ethercat_->set_slave_state(0, static_cast<uint16>(EthercatState::SafeOp));

    // 等待 master/slaves 真正進 SAFE_OP
    if (ethercat_->state_check(0, static_cast<uint16>(EthercatState::SafeOp), ethercat_->timeout_state()) != static_cast<uint16>(EthercatState::SafeOp))
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
    ethercat_->close();

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
    if (ethercat_) {
      ethercat_->close();
    }
    opened_ = false;
  }
}

void PanasonicEthercatInitializer::run_loop_impl(CyclicSession& session, AllMotors& motors)
{
  (void)session;
  (void)motors;
  const int cycle_ms = cycle_period_ms();
  const long long cycle_ns = static_cast<long long>(cycle_ms) * 1000LL * 1000LL;

  struct timespec ts;
  clock_gettime(CLOCK_MONOTONIC, &ts);
  struct timespec last_ts = ts;
  long long dt_sum_ns = 0;
  long long dt_min_ns = 0;
  long long dt_max_ns = 0;
  int dt_samples = 0;
  const int dt_log_interval = 500;
  while (running_)
  {
    ts.tv_nsec += cycle_ns;
    while (ts.tv_nsec >= 1000000000)
    {
        ts.tv_nsec -= 1000000000;
        ts.tv_sec += 1;
    }

    struct timespec now_ts;
    clock_gettime(CLOCK_MONOTONIC, &now_ts);
    long long dt_ns = (now_ts.tv_sec - last_ts.tv_sec) * 1000000000LL +
                      (now_ts.tv_nsec - last_ts.tv_nsec);
    last_ts = now_ts;
    if (dt_samples == 0) {
      dt_min_ns = dt_ns;
      dt_max_ns = dt_ns;
    } else {
      if (dt_ns < dt_min_ns) dt_min_ns = dt_ns;
      if (dt_ns > dt_max_ns) dt_max_ns = dt_ns;
    }
    dt_sum_ns += dt_ns;
    ++dt_samples;
    if (dt_samples >= dt_log_interval) {
      log_cycle_stats(dt_sum_ns, dt_min_ns, dt_max_ns, dt_samples);
      dt_sum_ns = 0;
      dt_min_ns = 0;
      dt_max_ns = 0;
      dt_samples = 0;
    }

    if (ethercat_) {
      ethercat_->set_pdo_data();
      ethercat_->get_pdo_data();
    }

    if (get_call_session_enabled()) {
      bool cycle_shutdown_request = false;
      session.run(motors, cycle_shutdown_request);
      if (cycle_shutdown_request) {
        session.setCallback(nullptr);
        notify_shutdown_requested();
      }
    }

    clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &ts, NULL);
  }
}
