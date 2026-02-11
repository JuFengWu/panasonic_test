#pragma once

#include "Ethercat.hpp"
#include "mu_ethercat/master.hpp"

#include <memory>
#include <vector>

class MuEthercat : public MyEthercat {
 public:
    MuEthercat() = default;
    ~MuEthercat() override;

    bool init(const char* ifname) override;
    void close() override;
    bool read_sdo_state(std::uint16_t slave,
                        std::uint16_t index,
                        std::uint8_t sub,
                        void* out,
                        int size) override;
    bool write_sdo_state(std::uint16_t slave,
                         std::uint16_t index,
                         std::uint8_t sub,
                         const void* data,
                         int size) override;
    std::string get_error_message() const override;
    bool scan_slaves() override;
    int config_pdo_mapping(void* io_map) override;
    bool setting_dc() override;
    bool read_ehtercat_state() override;
    bool write_ethercat_state(std::uint16_t slave) override;
    int state_check(std::uint16_t slave, std::uint16_t state, int timeout) override;
    bool set_pdo_data() override;
    bool get_pdo_data() override;
    bool close_ethercat() override;

    int get_slave_count() const override;
    bool set_slave_state(std::uint16_t slave, std::uint16_t state) override;
    std::uint16_t get_slave_state(std::uint16_t slave) const override;
    std::uint16_t get_slave_al_status(std::uint16_t slave) const override;
    std::uint8_t* get_slave_outputs(std::uint16_t slave) override;
    std::uint8_t* get_slave_inputs(std::uint16_t slave) override;
    int get_slave_obytes(std::uint16_t slave) const override;
    int get_slave_ibytes(std::uint16_t slave) const override;

    int timeout_state() const override;

    bool get_slave_info(std::uint16_t slave, std::uint32_t& vendor_id,
                        std::uint32_t& product_code,
                        std::uint32_t& revision,
                        std::uint32_t& serial) const;

    void set_process_mapping(std::uint32_t logical_addr,
                             std::uint16_t out_bytes_per_slave,
                             std::uint16_t in_bytes_per_slave);
    void set_dc_config(std::uint32_t sync0_cycle_ns,
                       std::uint32_t sync0_shift_ns = 0,
                       std::uint32_t sync1_cycle_ns = 0,
                       std::uint32_t sync1_shift_ns = 0);

 private:
    struct PdoConfig {
        std::uint16_t out_addr = 0;
        std::uint16_t out_len = 0;
        std::uint16_t in_addr = 0;
        std::uint16_t in_len = 0;
        std::size_t out_offset = 0;
        std::size_t in_offset = 0;
    };
    struct SlaveInfo {
        std::uint32_t vendor_id = 0;
        std::uint32_t product_code = 0;
        std::uint32_t revision = 0;
        std::uint32_t serial = 0;
    };

    struct MailboxConfig {
        std::uint16_t configadr = 0;
        std::uint16_t mbx_wo = 0;
        std::uint16_t mbx_ro = 0;
        std::uint16_t mbx_l = 0;
        std::uint16_t mbx_rl = 0;
        std::uint16_t mbx_proto = 0;
        std::uint8_t mbx_cnt = 0;
    };

    bool initialized_ = false;
    void* io_map_ = nullptr;
    int slave_count_ = 0;
    std::string last_error_;
    std::vector<std::uint8_t> outputs_;
    std::vector<std::uint8_t> inputs_;
    std::vector<SlaveInfo> slave_infos_;
    std::vector<std::uint16_t> slave_states_;
    std::vector<std::uint16_t> slave_al_status_;
    std::vector<MailboxConfig> mailboxes_;
    std::vector<PdoConfig> pdo_configs_;
    std::uint32_t logical_addr_ = 0;
    std::uint16_t out_bytes_per_slave_ = 0;
    std::uint16_t in_bytes_per_slave_ = 0;
    bool use_logical_pdo_ = false;
    bool use_block_sdo_ = false;
    std::uint32_t dc_sync0_cycle_ns_ = 0;
    std::uint32_t dc_sync0_shift_ns_ = 0;
    std::uint32_t dc_sync1_cycle_ns_ = 0;
    std::uint32_t dc_sync1_shift_ns_ = 0;

    std::unique_ptr<mu_ethercat::Master> master_;

    void set_error(const char* message);
    bool ensure_initialized();
    bool read_slave_info();
    bool configure_station_addresses();
    bool read_mailbox_config();
    bool mbx_write(std::uint16_t slave, const std::uint8_t* data, std::size_t size);
    bool mbx_read(std::uint16_t slave, std::vector<std::uint8_t>& out, int timeout_ms);
    bool eeprom_wait_not_busy(std::uint16_t aiadr, std::uint16_t& estat, int timeout_ms);
    bool eeprom_read_ap(std::uint16_t aiadr, std::uint16_t word_addr, std::uint32_t& out);
};
