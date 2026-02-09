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

bool SoemEthercat::setting_pdo_mapping(int slave_id) {
    (void)slave_id;
    last_error_.clear();
    int size = ec_config_map(io_map_.data());
    if (size <= 0) {
        last_error_ = make_soem_error_fallback("ec_config_map failed");
        return false;
    }
    return true;
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

bool SoemEthercat::write_ethercat_state() {
    last_error_.clear();
    int rc = ec_writestate(0);
    if (rc <= 0) {
        last_error_ = make_soem_error_fallback("ec_writestate failed");
        return false;
    }
    return true;
}

bool SoemEthercat::check_ethercat_state() {
    last_error_.clear();
    int state = ec_statecheck(0, EC_STATE_OPERATIONAL, EC_TIMEOUTSTATE);
    if (state != EC_STATE_OPERATIONAL) {
        last_error_ = make_soem_error_fallback("ec_statecheck failed");
        return false;
    }
    return true;
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
