#include "Ethercat.hpp"

extern "C" {
#include "ethercat.h"
}

namespace {
std::string make_soem_error_fallback(const char* fallback) {
    if (EcatError) {
        return std::string(ec_elist2string());
    }
    return std::string(fallback);
}
}

bool SoemEthercat::init(const char* ifname) {
    last_error_.clear();
    if (ifname == nullptr) {
        last_error_ = "ifname is null";
        return false;
    }
    if (!ec_init(ifname)) {
        last_error_ = make_soem_error_fallback("ec_init failed");
        return false;
    }
    return true;
}

void SoemEthercat::close() {
    ec_close();
}

bool SoemEthercat::read_sdo_state(std::uint16_t slave,
                                 std::uint16_t index,
                                 std::uint8_t sub,
                                 void* out,
                                 int size) {
    last_error_.clear();
    if (out == nullptr || size <= 0) {
        last_error_ = "SDO read output buffer invalid";
        return false;
    }

    int wkc = ec_SDOread(
        slave,
        index,
        sub,
        FALSE,
        &size,
        out,
        EC_TIMEOUTRXM
    );

    if (wkc <= 0) {
        last_error_ = make_soem_error_fallback("ec_SDOread failed");
        return false;
    }

    return true;
}

bool SoemEthercat::write_sdo_state(std::uint16_t slave,
                                  std::uint16_t index,
                                  std::uint8_t sub,
                                  const void* data,
                                  int size) {
    last_error_.clear();
    if (data == nullptr || size <= 0) {
        last_error_ = "SDO write data invalid";
        return false;
    }

    int wkc = ec_SDOwrite(
        slave,
        index,
        sub,
        FALSE,
        size,
        const_cast<void*>(data),
        EC_TIMEOUTRXM
    );

    if (wkc <= 0) {
        last_error_ = make_soem_error_fallback("ec_SDOwrite failed");
        return false;
    }

    return true;
}

std::string SoemEthercat::get_error_message() const {
    if (!last_error_.empty()) {
        return last_error_;
    }
    if (EcatError) {
        return std::string(ec_elist2string());
    }
    return std::string();
}

bool SoemEthercat::scan_slaves() {
    last_error_.clear();
    int count = ec_config_init(FALSE);
    if (count <= 0) {
        last_error_ = make_soem_error_fallback("ec_config_init failed");
        return false;
    }
    return true;
}

int SoemEthercat::config_pdo_mapping(void* io_map) {
    last_error_.clear();
    if (io_map == nullptr) {
        last_error_ = "io_map is null";
        return 0;
    }
    int size = ec_config_map(io_map);
    if (size <= 0) {
        last_error_ = make_soem_error_fallback("ec_config_map failed");
        return 0;
    }
    return size;
}

bool SoemEthercat::setting_dc() {
    last_error_.clear();
    if (!ec_configdc()) {
        last_error_ = make_soem_error_fallback("ec_configdc failed");
        return false;
    }
    return true;
}

bool SoemEthercat::read_ehtercat_state() {
    last_error_.clear();
    int rc = ec_readstate();
    if (rc <= 0) {
        last_error_ = make_soem_error_fallback("ec_readstate failed");
        return false;
    }
    return true;
}

bool SoemEthercat::write_ethercat_state(std::uint16_t slave) {
    last_error_.clear();
    int rc = ec_writestate(slave);
    if (rc <= 0) {
        last_error_ = make_soem_error_fallback("ec_writestate failed");
        return false;
    }
    return true;
}

int SoemEthercat::state_check(std::uint16_t slave, std::uint16_t state, int timeout) {
    return ec_statecheck(slave, state, timeout);
}

bool SoemEthercat::set_pdo_data() {
    last_error_.clear();
    int wkc = ec_send_processdata();
    if (wkc <= 0) {
        last_error_ = make_soem_error_fallback("ec_send_processdata failed");
        return false;
    }
    return true;
}

bool SoemEthercat::get_pdo_data() {
    last_error_.clear();
    int wkc = ec_receive_processdata(EC_TIMEOUTRET);
    if (wkc <= 0) {
        last_error_ = make_soem_error_fallback("ec_receive_processdata failed");
        return false;
    }
    return true;
}

bool SoemEthercat::close_ethercat() {
    last_error_.clear();
    ec_close();
    return true;
}

int SoemEthercat::get_slave_count() const {
    return ec_slavecount;
}

bool SoemEthercat::set_slave_state(std::uint16_t slave, std::uint16_t state) {
    if (slave > static_cast<std::uint16_t>(ec_slavecount)) {
        return false;
    }
    ec_slave[slave].state = state;
    return ec_writestate(slave) > 0;
}

std::uint16_t SoemEthercat::get_slave_state(std::uint16_t slave) const {
    if (slave > static_cast<std::uint16_t>(ec_slavecount)) {
        return 0;
    }
    return ec_slave[slave].state;
}

std::uint16_t SoemEthercat::get_slave_al_status(std::uint16_t slave) const {
    if (slave > static_cast<std::uint16_t>(ec_slavecount)) {
        return 0;
    }
    return ec_slave[slave].ALstatuscode;
}

std::uint8_t* SoemEthercat::get_slave_outputs(std::uint16_t slave) {
    if (slave > static_cast<std::uint16_t>(ec_slavecount)) {
        return nullptr;
    }
    return static_cast<std::uint8_t*>(ec_slave[slave].outputs);
}

std::uint8_t* SoemEthercat::get_slave_inputs(std::uint16_t slave) {
    if (slave > static_cast<std::uint16_t>(ec_slavecount)) {
        return nullptr;
    }
    return static_cast<std::uint8_t*>(ec_slave[slave].inputs);
}

int SoemEthercat::get_slave_obytes(std::uint16_t slave) const {
    if (slave > static_cast<std::uint16_t>(ec_slavecount)) {
        return 0;
    }
    return ec_slave[slave].Obytes;
}

int SoemEthercat::get_slave_ibytes(std::uint16_t slave) const {
    if (slave > static_cast<std::uint16_t>(ec_slavecount)) {
        return 0;
    }
    return ec_slave[slave].Ibytes;
}

int SoemEthercat::timeout_state() const {
    return EC_TIMEOUTSTATE;
}

const std::string& SoemEthercat::last_error() const {
    return last_error_;
}
