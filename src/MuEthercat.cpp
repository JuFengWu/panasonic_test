#include "MuEthercat.hpp"

#include "mu_ethercat/frame.hpp"
#include "mu_ethercat/master.hpp"

#include <algorithm>
#include <chrono>
#include <cstring>
#include <thread>

namespace {
constexpr int kTimeoutState = 2000;
constexpr int kDefaultPdoBytes = 8;
constexpr std::uint16_t kRegAlStatus = 0x0130;
constexpr std::uint16_t kRegAlControl = 0x0120;
constexpr std::uint16_t kRegAlStatusCode = 0x0134;
constexpr std::uint16_t kRegSmBase = 0x0800;
constexpr std::uint16_t kSmStride = 0x0008;
constexpr std::uint16_t kSm2 = 2;
constexpr std::uint16_t kSm3 = 3;
constexpr std::uint16_t kRegDcActivation = 0x0980;
constexpr std::uint16_t kRegDcSync0Cycle = 0x0990;
constexpr std::uint16_t kRegDcSync0Shift = 0x0994;
constexpr std::uint16_t kRegDcSync1Cycle = 0x0998;
constexpr std::uint16_t kRegDcSync1Shift = 0x099C;
constexpr std::uint16_t kRegFmmuBase = 0x0600;
constexpr std::uint16_t kFmmuStride = 0x0010;
constexpr int kScanTimeoutMs = 100;
constexpr int kStateTimeoutMs = 200;
constexpr int kMbxTimeoutMs = 500;
constexpr int kMbxRetryDelayMs = 2;
constexpr std::uint16_t kRegStAdr = 0x0010;
constexpr std::uint16_t kNodeOffset = 0x1000;
constexpr std::uint16_t kSiiRxMbxAdr = 0x0018;
constexpr std::uint16_t kSiiTxMbxAdr = 0x001A;
constexpr std::uint16_t kSiiMbxProto = 0x001C;
constexpr std::uint8_t kMbxTypeCoE = 0x03;
constexpr std::uint8_t kCoeSdoReq = 0x02;
constexpr std::uint8_t kCoeSdoRes = 0x03;
constexpr std::uint8_t kSdoCmdDownInit = 0x21;
constexpr std::uint8_t kSdoCmdDownExp4 = 0x23;
constexpr std::uint8_t kSdoCmdDownExp2 = 0x2B;
constexpr std::uint8_t kSdoCmdDownExp1 = 0x2F;
constexpr std::uint8_t kSdoCmdDownSeg = 0x00;
constexpr std::uint8_t kSdoCmdUpReq = 0x40;
constexpr std::uint8_t kSdoCmdUpExp4 = 0x43;
constexpr std::uint8_t kSdoCmdUpExp2 = 0x4B;
constexpr std::uint8_t kSdoCmdUpExp1 = 0x4F;
constexpr std::uint8_t kSdoCmdUpInitRes = 0x41;
constexpr std::uint8_t kSdoCmdUpSegReq = 0x60;
constexpr std::size_t kSdoSegmentSize = 7;
constexpr int kMbxWriteRetries = 3;
constexpr std::uint8_t kMbxCntMask = 0x07;
constexpr std::uint8_t kMbxCntStart = 1;
constexpr int kEepromTimeoutMs = 20;
constexpr int kEepromDelayUs = 200;
constexpr std::uint16_t kRegEepStat = 0x0502;
constexpr std::uint16_t kRegEepCtl = 0x0502;
constexpr std::uint16_t kRegEepDat = 0x0508;
constexpr std::uint16_t kEcmdNop = 0x0000;
constexpr std::uint16_t kEcmdRead = 0x0100;
constexpr std::uint16_t kEstatR64 = 0x0040;
constexpr std::uint16_t kEstatNack = 0x2000;
constexpr std::uint16_t kEstatBusy = 0x8000;
constexpr std::uint16_t kEstatEmask = 0x7800;
constexpr std::uint16_t kSiiManuf = 0x0008;
constexpr std::uint16_t kSiiId = 0x000A;
constexpr std::uint16_t kSiiRev = 0x000C;
constexpr std::uint16_t kSiiSerial = 0x000E;

inline std::uint16_t read_u16_le(const std::uint8_t* data) {
    return static_cast<std::uint16_t>(data[0] | (static_cast<std::uint16_t>(data[1]) << 8));
}

inline void write_u16_le(std::uint8_t* out, std::uint16_t value) {
    out[0] = static_cast<std::uint8_t>(value & 0xFF);
    out[1] = static_cast<std::uint8_t>((value >> 8) & 0xFF);
}

inline void write_u32_le(std::uint8_t* out, std::uint32_t value) {
    out[0] = static_cast<std::uint8_t>(value & 0xFF);
    out[1] = static_cast<std::uint8_t>((value >> 8) & 0xFF);
    out[2] = static_cast<std::uint8_t>((value >> 16) & 0xFF);
    out[3] = static_cast<std::uint8_t>((value >> 24) & 0xFF);
}

bool read_sm_config(mu_ethercat::Master& master,
                    std::uint16_t slave,
                    std::uint16_t sm_index,
                    std::uint16_t& start_addr,
                    std::uint16_t& length,
                    std::string& err) {
    const std::uint16_t ado = static_cast<std::uint16_t>(kRegSmBase + (sm_index * kSmStride));
    std::vector<std::uint8_t> response;
    if (!master.send_aprd(slave, ado, 4, response, kStateTimeoutMs)) {
        err = master.last_error();
        return false;
    }
    auto view = mu_ethercat::get_datagram_view(response.data(), response.size());
    if (view.payload == nullptr || view.payload_size < 4) {
        err = "invalid SM config response";
        return false;
    }
    start_addr = read_u16_le(view.payload);
    length = read_u16_le(view.payload + 2);
    return true;
}

bool write_u16(mu_ethercat::Master& master,
               std::uint16_t slave,
               std::uint16_t ado,
               std::uint16_t value,
               std::string& err) {
    std::uint8_t data[2] = {0, 0};
    write_u16_le(data, value);
    std::vector<std::uint8_t> response;
    if (!master.send_apwr(slave, ado, data, sizeof(data), response, kStateTimeoutMs)) {
        err = master.last_error();
        return false;
    }
    return true;
}

bool write_u32(mu_ethercat::Master& master,
               std::uint16_t slave,
               std::uint16_t ado,
               std::uint32_t value,
               std::string& err) {
    std::uint8_t data[4] = {0, 0, 0, 0};
    write_u32_le(data, value);
    std::vector<std::uint8_t> response;
    if (!master.send_apwr(slave, ado, data, sizeof(data), response, kStateTimeoutMs)) {
        err = master.last_error();
        return false;
    }
    return true;
}

bool write_fmmu(mu_ethercat::Master& master,
                std::uint16_t slave,
                std::uint16_t fmmu_index,
                std::uint32_t logical_start,
                std::uint16_t length,
                std::uint16_t physical_start,
                std::uint8_t fmmu_type,
                std::string& err) {
    std::uint8_t data[16] = {0};
    write_u32_le(data + 0, logical_start);
    write_u16_le(data + 4, length);
    data[6] = 0x00;
    data[7] = 0x07;
    write_u16_le(data + 8, physical_start);
    data[10] = 0x00;
    data[11] = fmmu_type;
    data[12] = 0x01;
    const std::uint16_t ado = static_cast<std::uint16_t>(kRegFmmuBase + fmmu_index * kFmmuStride);
    std::vector<std::uint8_t> response;
    if (!master.send_apwr(slave, ado, data, sizeof(data), response, kStateTimeoutMs)) {
        err = master.last_error();
        return false;
    }
    return true;
}
}

MuEthercat::~MuEthercat() = default;

void MuEthercat::set_error(const char* message) {
    last_error_.assign(message ? message : "unknown error");
}

bool MuEthercat::ensure_initialized() {
    if (!initialized_) {
        set_error("ethercat not initialized");
        return false;
    }
    if (!master_ || !master_->is_open()) {
        set_error("ethercat master not open");
        return false;
    }
    return true;
}

bool MuEthercat::init(const char* ifname) {
    last_error_.clear();
    if (ifname == nullptr || *ifname == '\0') {
        set_error("ifname is null or empty");
        return false;
    }

    if (!master_) {
        master_ = std::make_unique<mu_ethercat::Master>();
    }

    if (!master_->open(ifname)) {
        set_error(master_->last_error().c_str());
        return false;
    }

    initialized_ = true;
    io_map_ = nullptr;
    slave_count_ = 0;
    outputs_.clear();
    inputs_.clear();
    slave_states_.clear();
    mailboxes_.clear();
    pdo_configs_.clear();
    slave_al_status_.clear();
    return true;
}

void MuEthercat::close() {
    if (master_) {
        master_->close();
    }
    initialized_ = false;
    io_map_ = nullptr;
    slave_count_ = 0;
    outputs_.clear();
    inputs_.clear();
    slave_states_.clear();
    mailboxes_.clear();
    pdo_configs_.clear();
    slave_al_status_.clear();
}

bool MuEthercat::read_sdo_state(std::uint16_t slave,
                               std::uint16_t index,
                               std::uint8_t sub,
                               void* out,
                               int size) {
    last_error_.clear();
    if (!ensure_initialized()) {
        return false;
    }
    if (out == nullptr || size <= 0) {
        set_error("SDO read output buffer invalid");
        return false;
    }
    if (slave == 0 || slave > static_cast<std::uint16_t>(slave_count_)) {
        set_error("invalid slave id");
        return false;
    }
    if (mailboxes_.size() <= static_cast<std::size_t>(slave) || mailboxes_[slave].mbx_l == 0) {
        set_error("mailbox not configured");
        return false;
    }

    std::uint8_t sdo[8] = {0};
    sdo[0] = kSdoCmdUpReq;
    write_u16_le(sdo + 1, index);
    sdo[3] = sub;

    std::uint8_t coe[2] = {kCoeSdoReq, 0};

    std::vector<std::uint8_t> mbx(6 + sizeof(coe) + sizeof(sdo), 0);
    std::uint16_t len = static_cast<std::uint16_t>(sizeof(coe) + sizeof(sdo));
    write_u16_le(mbx.data(), len);
    write_u16_le(mbx.data() + 2, 0);
    mbx[4] = 0;
    mbx[5] = kMbxTypeCoE;
    std::memcpy(mbx.data() + 6, coe, sizeof(coe));
    std::memcpy(mbx.data() + 6 + sizeof(coe), sdo, sizeof(sdo));

    if (!mbx_write(slave, mbx.data(), mbx.size())) {
        return false;
    }

    std::vector<std::uint8_t> response;
    if (!mbx_read(slave, response, kMbxTimeoutMs)) {
        return false;
    }
    if (response.size() < 6 + 2 + 8) {
        set_error("invalid SDO response size");
        return false;
    }
    if (response[5] != kMbxTypeCoE || response[6] != kCoeSdoRes) {
        set_error("unexpected mailbox response");
        return false;
    }
    const std::uint8_t* sdo_res = response.data() + 8;
    if (sdo_res[1] != (index & 0xFF) || sdo_res[2] != ((index >> 8) & 0xFF) || sdo_res[3] != sub) {
        set_error("SDO response index mismatch");
        return false;
    }
    std::uint8_t cmd = sdo_res[0];
    int data_len = 0;
    if (cmd == kSdoCmdUpExp4) {
        data_len = 4;
    } else if (cmd == kSdoCmdUpExp2) {
        data_len = 2;
    } else if (cmd == kSdoCmdUpExp1) {
        data_len = 1;
    } else if ((cmd & 0xF0) == kSdoCmdUpInitRes) {
        // Segmented upload
        std::vector<std::uint8_t> buffer;
        buffer.reserve(static_cast<std::size_t>(size));
        bool toggle = false;
        while (true) {
            std::uint8_t seg_req[1] = {static_cast<std::uint8_t>(kSdoCmdUpSegReq | (toggle ? 0x10 : 0x00))};
            std::uint8_t coe_req[2] = {kCoeSdoReq, 0};
            std::vector<std::uint8_t> mbx_req(6 + sizeof(coe_req) + sizeof(seg_req), 0);
            std::uint16_t len = static_cast<std::uint16_t>(sizeof(coe_req) + sizeof(seg_req));
            write_u16_le(mbx_req.data(), len);
            write_u16_le(mbx_req.data() + 2, 0);
            mbx_req[4] = 0;
            mbx_req[5] = kMbxTypeCoE;
            std::memcpy(mbx_req.data() + 6, coe_req, sizeof(coe_req));
            std::memcpy(mbx_req.data() + 6 + sizeof(coe_req), seg_req, sizeof(seg_req));

            if (!mbx_write(slave, mbx_req.data(), mbx_req.size())) {
                return false;
            }

            std::vector<std::uint8_t> seg_res;
            if (!mbx_read(slave, seg_res, kMbxTimeoutMs)) {
                return false;
            }
            if (seg_res.size() < 6 + 2 + 1) {
                set_error("invalid SDO segment response");
                return false;
            }
            if (seg_res[5] != kMbxTypeCoE || seg_res[6] != kCoeSdoRes) {
                set_error("unexpected mailbox response");
                return false;
            }
            const std::uint8_t* seg_cmd = seg_res.data() + 8;
            std::uint8_t scmd = seg_cmd[0];
            if (((scmd & 0x10) != 0) != toggle) {
                set_error("SDO toggle bit mismatch");
                return false;
            }
            bool last = (scmd & 0x01) != 0;
            std::uint8_t n = (scmd >> 1) & 0x07;
            std::size_t seg_data_len = kSdoSegmentSize - n;
            if (seg_res.size() < 6 + 2 + 1 + seg_data_len) {
                set_error("invalid SDO segment length");
                return false;
            }
            const std::uint8_t* seg_data = seg_res.data() + 9;
            buffer.insert(buffer.end(), seg_data, seg_data + seg_data_len);
            if (last) {
                break;
            }
            toggle = !toggle;
        }

        int copy_len = std::min<int>(size, static_cast<int>(buffer.size()));
        std::memcpy(out, buffer.data(), static_cast<std::size_t>(copy_len));
        return true;
    } else {
        set_error("unsupported SDO response");
        return false;
    }
    int copy_len = std::min(size, data_len);
    std::memcpy(out, sdo_res + 4, static_cast<std::size_t>(copy_len));
    return true;
}

bool MuEthercat::write_sdo_state(std::uint16_t slave,
                                std::uint16_t index,
                                std::uint8_t sub,
                                const void* data,
                                int size) {
    last_error_.clear();
    if (!ensure_initialized()) {
        return false;
    }
    if (data == nullptr || size <= 0) {
        set_error("SDO write data invalid");
        return false;
    }
    if (slave == 0 || slave > static_cast<std::uint16_t>(slave_count_)) {
        set_error("invalid slave id");
        return false;
    }
    if (mailboxes_.size() <= static_cast<std::size_t>(slave) || mailboxes_[slave].mbx_l == 0) {
        set_error("mailbox not configured");
        return false;
    }
    if (size <= 0) {
        set_error("unsupported SDO write size");
        return false;
    }

    std::uint8_t cmd = kSdoCmdDownInit;
    if (size == 4) {
        cmd = kSdoCmdDownExp4;
    } else if (size == 2) {
        cmd = kSdoCmdDownExp2;
    } else if (size == 1) {
        cmd = kSdoCmdDownExp1;
    }

    std::uint8_t sdo[8] = {0};
    sdo[0] = cmd;
    write_u16_le(sdo + 1, index);
    sdo[3] = sub;
    if (size <= 4) {
        std::memcpy(sdo + 4, data, static_cast<std::size_t>(size));
    } else {
        write_u16_le(sdo + 4, static_cast<std::uint16_t>(size & 0xFFFF));
        write_u16_le(sdo + 6, static_cast<std::uint16_t>((size >> 16) & 0xFFFF));
    }

    std::uint8_t coe[2] = {kCoeSdoReq, 0};

    std::vector<std::uint8_t> mbx(6 + sizeof(coe) + sizeof(sdo), 0);
    std::uint16_t len = static_cast<std::uint16_t>(sizeof(coe) + sizeof(sdo));
    write_u16_le(mbx.data(), len);
    write_u16_le(mbx.data() + 2, 0);
    mbx[4] = 0;
    mbx[5] = kMbxTypeCoE;
    std::memcpy(mbx.data() + 6, coe, sizeof(coe));
    std::memcpy(mbx.data() + 6 + sizeof(coe), sdo, sizeof(sdo));

    if (!mbx_write(slave, mbx.data(), mbx.size())) {
        return false;
    }

    std::vector<std::uint8_t> response;
    if (!mbx_read(slave, response, kMbxTimeoutMs)) {
        return false;
    }
    if (response.size() < 6 + 2 + 8) {
        set_error("invalid SDO response size");
        return false;
    }
    if (response[5] != kMbxTypeCoE || response[6] != kCoeSdoRes) {
        set_error("unexpected mailbox response");
        return false;
    }
    const std::uint8_t* sdo_res = response.data() + 8;
    if (sdo_res[1] != (index & 0xFF) || sdo_res[2] != ((index >> 8) & 0xFF) || sdo_res[3] != sub) {
        set_error("SDO response index mismatch");
        return false;
    }
    if (size <= 4) {
        return true;
    }

    // Segmented download
    const std::uint8_t* src = static_cast<const std::uint8_t*>(data);
    std::size_t remaining = static_cast<std::size_t>(size);
    bool toggle = false;
    while (remaining > 0) {
        std::size_t seg_len = std::min<std::size_t>(kSdoSegmentSize, remaining);
        std::uint8_t n = static_cast<std::uint8_t>(kSdoSegmentSize - seg_len);
        bool last = (remaining <= kSdoSegmentSize);
        std::uint8_t seg_cmd = static_cast<std::uint8_t>(kSdoCmdDownSeg |
                                                         (toggle ? 0x10 : 0x00) |
                                                         (n << 1) |
                                                         (last ? 0x01 : 0x00));

        std::uint8_t coe_req[2] = {kCoeSdoReq, 0};
        std::vector<std::uint8_t> mbx_req(6 + sizeof(coe_req) + 1 + seg_len, 0);
        std::uint16_t len = static_cast<std::uint16_t>(sizeof(coe_req) + 1 + seg_len);
        write_u16_le(mbx_req.data(), len);
        write_u16_le(mbx_req.data() + 2, 0);
        mbx_req[4] = 0;
        mbx_req[5] = kMbxTypeCoE;
        mbx_req[6] = coe_req[0];
        mbx_req[7] = coe_req[1];
        mbx_req[8] = seg_cmd;
        std::memcpy(mbx_req.data() + 9, src, seg_len);

        if (!mbx_write(slave, mbx_req.data(), mbx_req.size())) {
            return false;
        }
        std::vector<std::uint8_t> seg_res;
        if (!mbx_read(slave, seg_res, kMbxTimeoutMs)) {
            return false;
        }
        if (seg_res.size() < 6 + 2 + 1) {
            set_error("invalid SDO segment response");
            return false;
        }
        if (seg_res[5] != kMbxTypeCoE || seg_res[6] != kCoeSdoRes) {
            set_error("unexpected mailbox response");
            return false;
        }
        std::uint8_t scmd = seg_res[8];
        if (((scmd & 0x10) != 0) != toggle) {
            set_error("SDO toggle bit mismatch");
            return false;
        }
        toggle = !toggle;
        src += seg_len;
        remaining -= seg_len;
    }
    return true;
}

std::string MuEthercat::get_error_message() const {
    return last_error_;
}

bool MuEthercat::scan_slaves() {
    last_error_.clear();
    if (!ensure_initialized()) {
        return false;
    }

    std::vector<std::uint8_t> response;
    if (!master_->send_brd(0x0000, kRegAlStatus, 2, response, kScanTimeoutMs)) {
        set_error(master_->last_error().c_str());
        return false;
    }

    auto view = mu_ethercat::get_datagram_view(response.data(), response.size());
    if (view.wkc == nullptr) {
        set_error("invalid scan response");
        return false;
    }

    std::uint16_t wkc = *view.wkc;
    if (wkc == 0) {
        set_error("no slaves detected");
        return false;
    }

    slave_count_ = static_cast<int>(wkc);
    outputs_.assign(static_cast<std::size_t>(slave_count_) * kDefaultPdoBytes, 0);
    inputs_.assign(static_cast<std::size_t>(slave_count_) * kDefaultPdoBytes, 0);
    slave_infos_.assign(static_cast<std::size_t>(slave_count_ + 1), {});
    slave_states_.assign(static_cast<std::size_t>(slave_count_ + 1), 0);
    slave_al_status_.assign(static_cast<std::size_t>(slave_count_ + 1), 0);
    mailboxes_.assign(static_cast<std::size_t>(slave_count_ + 1), {});
    pdo_configs_.assign(static_cast<std::size_t>(slave_count_ + 1), {});

    if (!read_slave_info()) {
        return false;
    }
    if (!configure_station_addresses()) {
        return false;
    }
    if (!read_mailbox_config()) {
        return false;
    }
    return true;
}

int MuEthercat::config_pdo_mapping(void* io_map) {
    last_error_.clear();
    if (!ensure_initialized()) {
        return 0;
    }
    if (io_map == nullptr) {
        set_error("io_map is null");
        return 0;
    }
    io_map_ = io_map;

    std::size_t out_total = 0;
    std::size_t in_total = 0;
    if (pdo_configs_.size() < static_cast<std::size_t>(slave_count_ + 1)) {
        pdo_configs_.assign(static_cast<std::size_t>(slave_count_ + 1), {});
    }

    std::vector<std::uint16_t> sm2_addr(slave_count_ + 1, 0);
    std::vector<std::uint16_t> sm2_len(slave_count_ + 1, 0);
    std::vector<std::uint16_t> sm3_addr(slave_count_ + 1, 0);
    std::vector<std::uint16_t> sm3_len(slave_count_ + 1, 0);

    for (int i = 1; i <= slave_count_; ++i) {
        if (!read_sm_config(*master_, static_cast<std::uint16_t>(i), kSm2, sm2_addr[i], sm2_len[i], last_error_)) {
            return 0;
        }
        if (!read_sm_config(*master_, static_cast<std::uint16_t>(i), kSm3, sm3_addr[i], sm3_len[i], last_error_)) {
            return 0;
        }
    }

    if (use_logical_pdo_) {
        out_total = static_cast<std::size_t>(slave_count_) * out_bytes_per_slave_;
        in_total = static_cast<std::size_t>(slave_count_) * in_bytes_per_slave_;
        for (int i = 1; i <= slave_count_; ++i) {
            auto& cfg = pdo_configs_[static_cast<std::size_t>(i)];
            cfg.out_addr = sm2_addr[i];
            cfg.out_len = out_bytes_per_slave_;
            cfg.in_addr = sm3_addr[i];
            cfg.in_len = in_bytes_per_slave_;
            cfg.out_offset = static_cast<std::size_t>(i - 1) * out_bytes_per_slave_;
            cfg.in_offset = static_cast<std::size_t>(i - 1) * in_bytes_per_slave_;
        }

        std::uint32_t logical_out = logical_addr_;
        std::uint32_t logical_in = logical_addr_ + static_cast<std::uint32_t>(out_total);
        for (int i = 1; i <= slave_count_; ++i) {
            if (out_bytes_per_slave_ > 0) {
                if (!write_fmmu(*master_,
                                static_cast<std::uint16_t>(i),
                                0,
                                logical_out,
                                out_bytes_per_slave_,
                                sm2_addr[i],
                                0x01,
                                last_error_)) {
                    return 0;
                }
            }
            if (in_bytes_per_slave_ > 0) {
                if (!write_fmmu(*master_,
                                static_cast<std::uint16_t>(i),
                                1,
                                logical_in,
                                in_bytes_per_slave_,
                                sm3_addr[i],
                                0x02,
                                last_error_)) {
                    return 0;
                }
            }
            logical_out += out_bytes_per_slave_;
            logical_in += in_bytes_per_slave_;
        }
    } else {
        // Default to logical mapping using discovered SM lengths for SOEM-like behavior.
        logical_addr_ = 0x1000;
        use_logical_pdo_ = true;

        for (int i = 1; i <= slave_count_; ++i) {
            auto& cfg = pdo_configs_[static_cast<std::size_t>(i)];
            cfg.out_addr = sm2_addr[i];
            cfg.out_len = sm2_len[i];
            cfg.in_addr = sm3_addr[i];
            cfg.in_len = sm3_len[i];
            cfg.out_offset = out_total;
            cfg.in_offset = in_total;
            out_total += sm2_len[i];
            in_total += sm3_len[i];
        }

        std::uint32_t logical_out = logical_addr_;
        std::uint32_t logical_in = logical_addr_ + static_cast<std::uint32_t>(out_total);
        for (int i = 1; i <= slave_count_; ++i) {
            const auto& cfg = pdo_configs_[static_cast<std::size_t>(i)];
            if (cfg.out_len > 0) {
                if (!write_fmmu(*master_,
                                static_cast<std::uint16_t>(i),
                                0,
                                logical_out,
                                cfg.out_len,
                                sm2_addr[i],
                                0x01,
                                last_error_)) {
                    return 0;
                }
            }
            if (cfg.in_len > 0) {
                if (!write_fmmu(*master_,
                                static_cast<std::uint16_t>(i),
                                1,
                                logical_in,
                                cfg.in_len,
                                sm3_addr[i],
                                0x02,
                                last_error_)) {
                    return 0;
                }
            }
            logical_out += cfg.out_len;
            logical_in += cfg.in_len;
        }
    }

    outputs_.assign(out_total, 0);
    inputs_.assign(in_total, 0);
    return static_cast<int>(outputs_.size() + inputs_.size());
}

bool MuEthercat::setting_dc() {
    last_error_.clear();
    if (!ensure_initialized()) {
        return false;
    }
    if (slave_count_ <= 0) {
        return true;
    }
    if (dc_sync0_cycle_ns_ == 0) {
        return true;
    }
    for (int i = 1; i <= slave_count_; ++i) {
        std::uint16_t activation = 0x0001;
        if (dc_sync1_cycle_ns_ > 0) {
            activation |= 0x0002;
        }
        if (!write_u16(*master_, static_cast<std::uint16_t>(i), kRegDcActivation, activation, last_error_)) {
            return false;
        }
        if (!write_u32(*master_, static_cast<std::uint16_t>(i), kRegDcSync0Cycle, dc_sync0_cycle_ns_, last_error_)) {
            return false;
        }
        if (!write_u32(*master_, static_cast<std::uint16_t>(i), kRegDcSync0Shift, dc_sync0_shift_ns_, last_error_)) {
            return false;
        }
        if (dc_sync1_cycle_ns_ > 0) {
            if (!write_u32(*master_, static_cast<std::uint16_t>(i), kRegDcSync1Cycle, dc_sync1_cycle_ns_, last_error_)) {
                return false;
            }
            if (!write_u32(*master_, static_cast<std::uint16_t>(i), kRegDcSync1Shift, dc_sync1_shift_ns_, last_error_)) {
                return false;
            }
        }
    }
    return true;
}

bool MuEthercat::read_ehtercat_state() {
    last_error_.clear();
    if (!ensure_initialized()) {
        return false;
    }
    if (slave_count_ <= 0) {
        return true;
    }

    for (int i = 1; i <= slave_count_; ++i) {
        std::vector<std::uint8_t> response;
        if (!master_->send_aprd(static_cast<std::uint16_t>(i), kRegAlStatus, 2, response, kStateTimeoutMs)) {
            set_error(master_->last_error().c_str());
            return false;
        }
        auto view = mu_ethercat::get_datagram_view(response.data(), response.size());
        if (view.payload == nullptr || view.payload_size < 2) {
            set_error("invalid AL status response");
            return false;
        }
        if (slave_states_.size() <= static_cast<std::size_t>(i)) {
            slave_states_.resize(static_cast<std::size_t>(i + 1), 0);
        }
        slave_states_[static_cast<std::size_t>(i)] = read_u16_le(view.payload);

        std::vector<std::uint8_t> al_response;
        if (!master_->send_aprd(static_cast<std::uint16_t>(i), kRegAlStatusCode, 2, al_response, kStateTimeoutMs)) {
            set_error(master_->last_error().c_str());
            return false;
        }
        auto al_view = mu_ethercat::get_datagram_view(al_response.data(), al_response.size());
        if (al_view.payload == nullptr || al_view.payload_size < 2) {
            set_error("invalid AL status code response");
            return false;
        }
        if (slave_al_status_.size() <= static_cast<std::size_t>(i)) {
            slave_al_status_.resize(static_cast<std::size_t>(i + 1), 0);
        }
        slave_al_status_[static_cast<std::size_t>(i)] = read_u16_le(al_view.payload);
    }

    return true;
}

bool MuEthercat::write_ethercat_state(std::uint16_t slave) {
    last_error_.clear();
    if (!ensure_initialized()) {
        return false;
    }

    if (slave == 0 || slave > static_cast<std::uint16_t>(slave_count_)) {
        set_error("invalid slave id");
        return false;
    }
    if (slave_states_.empty()) {
        set_error("slave states not initialized");
        return false;
    }

    std::uint16_t state = slave_states_[static_cast<std::size_t>(slave)];
    std::uint8_t data[2] = {0, 0};
    write_u16_le(data, state);
    std::vector<std::uint8_t> response;
    if (!master_->send_apwr(slave, kRegAlControl, data, sizeof(data), response, kStateTimeoutMs)) {
        set_error(master_->last_error().c_str());
        return false;
    }
    return true;
}

int MuEthercat::state_check(std::uint16_t slave, std::uint16_t state, int timeout) {
    if (!initialized_) {
        return 0;
    }
    if (slave > static_cast<std::uint16_t>(slave_count_)) {
        return 0;
    }
    if (slave_count_ == 0) {
        return 0;
    }
    const int timeout_ms = (timeout > 0) ? timeout : kTimeoutState;
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout_ms);
    while (std::chrono::steady_clock::now() < deadline) {
        bool all_match = true;
        int start = (slave == 0) ? 1 : slave;
        int end = (slave == 0) ? slave_count_ : slave;
        for (int i = start; i <= end; ++i) {
            std::vector<std::uint8_t> response;
            if (!master_->send_aprd(static_cast<std::uint16_t>(i), kRegAlStatus, 2, response, kStateTimeoutMs)) {
                all_match = false;
                break;
            }
            auto view = mu_ethercat::get_datagram_view(response.data(), response.size());
            if (view.payload == nullptr || view.payload_size < 2) {
                all_match = false;
                break;
            }
            std::uint16_t current = read_u16_le(view.payload);
            if (slave_states_.size() <= static_cast<std::size_t>(i)) {
                slave_states_.resize(static_cast<std::size_t>(i + 1), 0);
            }
            slave_states_[static_cast<std::size_t>(i)] = current;
            if (current != state) {
                all_match = false;
            }
        }
        if (all_match) {
            return static_cast<int>(state);
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    return 0;
}

bool MuEthercat::set_pdo_data() {
    last_error_.clear();
    if (!ensure_initialized()) {
        return false;
    }
    if (outputs_.empty() && inputs_.empty()) {
        set_error("process data buffers not configured");
        return false;
    }
    if (pdo_configs_.empty()) {
        set_error("process mapping not configured");
        return false;
    }

    if (use_logical_pdo_) {
        std::vector<std::uint8_t> io_data;
        io_data.reserve(outputs_.size() + inputs_.size());
        io_data.insert(io_data.end(), outputs_.begin(), outputs_.end());
        io_data.insert(io_data.end(), inputs_.begin(), inputs_.end());
        std::vector<std::uint8_t> response;
        if (!master_->send_lrw(logical_addr_, io_data.data(), io_data.size(), response, kStateTimeoutMs)) {
            set_error(master_->last_error().c_str());
            return false;
        }
        auto view = mu_ethercat::get_datagram_view(response.data(), response.size());
        if (view.payload == nullptr || view.payload_size < io_data.size()) {
            set_error("invalid LRW response");
            return false;
        }
        if (!inputs_.empty()) {
            std::memcpy(inputs_.data(),
                        view.payload + outputs_.size(),
                        std::min(inputs_.size(), view.payload_size - outputs_.size()));
        }
        return true;
    }

    for (int i = 1; i <= slave_count_; ++i) {
        const auto& cfg = pdo_configs_[static_cast<std::size_t>(i)];
        if (cfg.out_len == 0) {
            continue;
        }
        const std::uint8_t* out = outputs_.data() + cfg.out_offset;
        std::vector<std::uint8_t> response;
        if (!master_->send_apwr(static_cast<std::uint16_t>(i),
                                cfg.out_addr,
                                out,
                                cfg.out_len,
                                response,
                                kStateTimeoutMs)) {
            set_error(master_->last_error().c_str());
            return false;
        }
    }
    return true;
}

bool MuEthercat::get_pdo_data() {
    last_error_.clear();
    if (!ensure_initialized()) {
        return false;
    }
    if (inputs_.empty()) {
        return true;
    }
    if (pdo_configs_.empty()) {
        set_error("process mapping not configured");
        return false;
    }
    if (use_logical_pdo_) {
        if (outputs_.empty() && inputs_.empty()) {
            return true;
        }
        std::vector<std::uint8_t> io_data;
        io_data.reserve(outputs_.size() + inputs_.size());
        io_data.insert(io_data.end(), outputs_.begin(), outputs_.end());
        io_data.insert(io_data.end(), inputs_.begin(), inputs_.end());
        std::vector<std::uint8_t> response;
        if (!master_->send_lrw(logical_addr_, io_data.data(), io_data.size(), response, kStateTimeoutMs)) {
            set_error(master_->last_error().c_str());
            return false;
        }
        auto view = mu_ethercat::get_datagram_view(response.data(), response.size());
        if (view.payload == nullptr || view.payload_size < io_data.size()) {
            set_error("invalid LRW response");
            return false;
        }
        if (!inputs_.empty()) {
            std::memcpy(inputs_.data(),
                        view.payload + outputs_.size(),
                        std::min(inputs_.size(), view.payload_size - outputs_.size()));
        }
        return true;
    }
    for (int i = 1; i <= slave_count_; ++i) {
        const auto& cfg = pdo_configs_[static_cast<std::size_t>(i)];
        if (cfg.in_len == 0) {
            continue;
        }
        std::vector<std::uint8_t> response;
        if (!master_->send_aprd(static_cast<std::uint16_t>(i),
                                cfg.in_addr,
                                cfg.in_len,
                                response,
                                kStateTimeoutMs)) {
            set_error(master_->last_error().c_str());
            return false;
        }
        auto view = mu_ethercat::get_datagram_view(response.data(), response.size());
        if (view.payload == nullptr || view.payload_size < cfg.in_len) {
            set_error("invalid APRD response");
            return false;
        }
        std::memcpy(inputs_.data() + cfg.in_offset, view.payload, cfg.in_len);
    }
    return true;
}

bool MuEthercat::close_ethercat() {
    last_error_.clear();
    close();
    return true;
}

int MuEthercat::get_slave_count() const {
    return slave_count_;
}

bool MuEthercat::set_slave_state(std::uint16_t slave, std::uint16_t state) {
    if (!initialized_) {
        return false;
    }
    std::uint8_t data[2] = {0, 0};
    write_u16_le(data, state);
    std::vector<std::uint8_t> response;
    if (slave == 0) {
        if (!master_->send_bwr(0, kRegAlControl, data, sizeof(data), response, kStateTimeoutMs)) {
            return false;
        }
        for (int i = 1; i <= slave_count_; ++i) {
            if (slave_states_.size() <= static_cast<std::size_t>(i)) {
                slave_states_.resize(static_cast<std::size_t>(i + 1), 0);
            }
            slave_states_[static_cast<std::size_t>(i)] = state;
        }
        return true;
    }
    if (slave > static_cast<std::uint16_t>(slave_count_)) {
        return false;
    }
    if (slave_states_.size() <= static_cast<std::size_t>(slave)) {
        slave_states_.resize(static_cast<std::size_t>(slave + 1), 0);
    }
    slave_states_[static_cast<std::size_t>(slave)] = state;
    if (!master_->send_apwr(slave, kRegAlControl, data, sizeof(data), response, kStateTimeoutMs)) {
        return false;
    }
    return true;
}

std::uint16_t MuEthercat::get_slave_state(std::uint16_t slave) const {
    if (slave == 0 || slave > static_cast<std::uint16_t>(slave_count_)) {
        return 0;
    }
    if (slave_states_.empty()) {
        return 0;
    }
    return slave_states_[static_cast<std::size_t>(slave)];
}

std::uint16_t MuEthercat::get_slave_al_status(std::uint16_t slave) const {
    if (slave == 0 || slave > static_cast<std::uint16_t>(slave_count_)) {
        return 0;
    }
    if (slave_al_status_.empty()) {
        return 0;
    }
    return slave_al_status_[static_cast<std::size_t>(slave)];
}

std::uint8_t* MuEthercat::get_slave_outputs(std::uint16_t slave) {
    if (slave == 0 || slave > static_cast<std::uint16_t>(slave_count_)) {
        return nullptr;
    }
    if (pdo_configs_.empty() || outputs_.empty()) {
        return nullptr;
    }
    const auto& cfg = pdo_configs_[static_cast<std::size_t>(slave)];
    if (cfg.out_len == 0) {
        return nullptr;
    }
    return outputs_.data() + cfg.out_offset;
}

std::uint8_t* MuEthercat::get_slave_inputs(std::uint16_t slave) {
    if (slave == 0 || slave > static_cast<std::uint16_t>(slave_count_)) {
        return nullptr;
    }
    if (pdo_configs_.empty() || inputs_.empty()) {
        return nullptr;
    }
    const auto& cfg = pdo_configs_[static_cast<std::size_t>(slave)];
    if (cfg.in_len == 0) {
        return nullptr;
    }
    return inputs_.data() + cfg.in_offset;
}

int MuEthercat::get_slave_obytes(std::uint16_t slave) const {
    if (slave == 0 || slave > static_cast<std::uint16_t>(slave_count_)) {
        return 0;
    }
    if (pdo_configs_.empty()) {
        return 0;
    }
    return static_cast<int>(pdo_configs_[static_cast<std::size_t>(slave)].out_len);
}

int MuEthercat::get_slave_ibytes(std::uint16_t slave) const {
    if (slave == 0 || slave > static_cast<std::uint16_t>(slave_count_)) {
        return 0;
    }
    if (pdo_configs_.empty()) {
        return 0;
    }
    return static_cast<int>(pdo_configs_[static_cast<std::size_t>(slave)].in_len);
}

int MuEthercat::timeout_state() const {
    return kTimeoutState;
}

bool MuEthercat::get_slave_info(std::uint16_t slave,
                                std::uint32_t& vendor_id,
                                std::uint32_t& product_code,
                                std::uint32_t& revision,
                                std::uint32_t& serial) const {
    if (slave == 0 || slave > static_cast<std::uint16_t>(slave_count_)) {
        return false;
    }
    if (slave_infos_.empty()) {
        return false;
    }
    const auto& info = slave_infos_[static_cast<std::size_t>(slave)];
    vendor_id = info.vendor_id;
    product_code = info.product_code;
    revision = info.revision;
    serial = info.serial;
    return true;
}

void MuEthercat::set_process_mapping(std::uint32_t logical_addr,
                                     std::uint16_t out_bytes_per_slave,
                                     std::uint16_t in_bytes_per_slave) {
    logical_addr_ = logical_addr;
    out_bytes_per_slave_ = out_bytes_per_slave;
    in_bytes_per_slave_ = in_bytes_per_slave;
    use_logical_pdo_ = (logical_addr_ != 0 && out_bytes_per_slave_ > 0 && in_bytes_per_slave_ > 0);
}

void MuEthercat::set_dc_config(std::uint32_t sync0_cycle_ns,
                               std::uint32_t sync0_shift_ns,
                               std::uint32_t sync1_cycle_ns,
                               std::uint32_t sync1_shift_ns) {
    dc_sync0_cycle_ns_ = sync0_cycle_ns;
    dc_sync0_shift_ns_ = sync0_shift_ns;
    dc_sync1_cycle_ns_ = sync1_cycle_ns;
    dc_sync1_shift_ns_ = sync1_shift_ns;
}

bool MuEthercat::read_slave_info() {
    if (!master_) {
        set_error("master not available");
        return false;
    }

    for (int i = 1; i <= slave_count_; ++i) {
        std::uint16_t aiadr = static_cast<std::uint16_t>(i);
        std::uint32_t vendor = 0;
        std::uint32_t product = 0;
        std::uint32_t revision = 0;
        std::uint32_t serial = 0;

        if (!eeprom_read_ap(aiadr, kSiiManuf, vendor)) {
            set_error("read vendor id failed");
            return false;
        }
        if (!eeprom_read_ap(aiadr, kSiiId, product)) {
            set_error("read product id failed");
            return false;
        }
        if (!eeprom_read_ap(aiadr, kSiiRev, revision)) {
            set_error("read revision failed");
            return false;
        }
        if (!eeprom_read_ap(aiadr, kSiiSerial, serial)) {
            set_error("read serial failed");
            return false;
        }

        auto& info = slave_infos_[static_cast<std::size_t>(i)];
        info.vendor_id = vendor;
        info.product_code = product;
        info.revision = revision;
        info.serial = serial;
    }

    return true;
}

bool MuEthercat::configure_station_addresses() {
    if (!master_) {
        set_error("master not available");
        return false;
    }
    if (mailboxes_.size() < static_cast<std::size_t>(slave_count_ + 1)) {
        mailboxes_.assign(static_cast<std::size_t>(slave_count_ + 1), {});
    }

    for (int i = 1; i <= slave_count_; ++i) {
        std::uint16_t aiadr = static_cast<std::uint16_t>(i);
        std::uint16_t configadr = static_cast<std::uint16_t>(kNodeOffset + i);
        std::uint8_t data[2] = {0, 0};
        write_u16_le(data, configadr);
        std::vector<std::uint8_t> response;
        if (!master_->send_apwr(aiadr, kRegStAdr, data, sizeof(data), response, kScanTimeoutMs)) {
            set_error(master_->last_error().c_str());
            return false;
        }
        mailboxes_[static_cast<std::size_t>(i)].configadr = configadr;
    }
    return true;
}

bool MuEthercat::read_mailbox_config() {
    for (int i = 1; i <= slave_count_; ++i) {
        std::uint16_t aiadr = static_cast<std::uint16_t>(i);
        std::uint32_t rx = 0;
        std::uint32_t tx = 0;
        std::uint32_t proto = 0;

        if (!eeprom_read_ap(aiadr, kSiiRxMbxAdr, rx)) {
            set_error("read RX mailbox failed");
            return false;
        }
        if (!eeprom_read_ap(aiadr, kSiiTxMbxAdr, tx)) {
            set_error("read TX mailbox failed");
            return false;
        }
        if (!eeprom_read_ap(aiadr, kSiiMbxProto, proto)) {
            set_error("read mailbox proto failed");
            return false;
        }

        auto& mbx = mailboxes_[static_cast<std::size_t>(i)];
        mbx.mbx_wo = static_cast<std::uint16_t>(rx & 0xFFFF);
        mbx.mbx_l = static_cast<std::uint16_t>((rx >> 16) & 0xFFFF);
        mbx.mbx_ro = static_cast<std::uint16_t>(tx & 0xFFFF);
        mbx.mbx_rl = static_cast<std::uint16_t>((tx >> 16) & 0xFFFF);
        if (mbx.mbx_rl == 0) {
            mbx.mbx_rl = mbx.mbx_l;
        }
        mbx.mbx_proto = static_cast<std::uint16_t>(proto & 0xFFFF);
    }
    return true;
}

bool MuEthercat::mbx_write(std::uint16_t slave, const std::uint8_t* data, std::size_t size) {
    if (slave == 0 || slave > static_cast<std::uint16_t>(slave_count_)) {
        set_error("invalid slave id");
        return false;
    }
    const auto& mbx = mailboxes_[static_cast<std::size_t>(slave)];
    if (mbx.mbx_l == 0 || mbx.configadr == 0) {
        set_error("mailbox not configured");
        return false;
    }
    if (size > mbx.mbx_l) {
        set_error("mailbox write size exceeds mailbox length");
        return false;
    }
    if (size < 6) {
        set_error("mailbox frame too small");
        return false;
    }
    auto& mbx_state = mailboxes_[static_cast<std::size_t>(slave)];
    if (mbx_state.mbx_cnt == 0) {
        mbx_state.mbx_cnt = kMbxCntStart;
    }

    std::vector<std::uint8_t> frame(data, data + size);
    frame[4] = static_cast<std::uint8_t>((frame[4] & ~kMbxCntMask) | (mbx_state.mbx_cnt & kMbxCntMask));

    for (int attempt = 0; attempt < kMbxWriteRetries; ++attempt) {
        std::vector<std::uint8_t> response;
        if (master_->send_fpwr(mbx.configadr, mbx.mbx_wo, frame.data(), frame.size(), response, kMbxTimeoutMs)) {
            mbx_state.mbx_cnt = static_cast<std::uint8_t>((mbx_state.mbx_cnt % kMbxCntMask) + 1);
            return true;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(kMbxRetryDelayMs));
    }
    set_error(master_->last_error().c_str());
    return false;
}

bool MuEthercat::mbx_read(std::uint16_t slave, std::vector<std::uint8_t>& out, int timeout_ms) {
    out.clear();
    if (slave == 0 || slave > static_cast<std::uint16_t>(slave_count_)) {
        set_error("invalid slave id");
        return false;
    }
    const auto& mbx = mailboxes_[static_cast<std::size_t>(slave)];
    if (mbx.mbx_rl == 0 || mbx.configadr == 0) {
        set_error("mailbox not configured");
        return false;
    }

    auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout_ms);
    while (std::chrono::steady_clock::now() < deadline) {
        std::vector<std::uint8_t> response;
        if (!master_->send_fprd(mbx.configadr, mbx.mbx_ro, mbx.mbx_rl, response, timeout_ms)) {
            set_error(master_->last_error().c_str());
            return false;
        }
        auto view = mu_ethercat::get_datagram_view(response.data(), response.size());
        if (view.payload == nullptr || view.payload_size < 6) {
            set_error("invalid mailbox read response");
            return false;
        }
        std::uint16_t length = read_u16_le(view.payload);
        if (length > 0 && length + 6 <= view.payload_size) {
            out.assign(view.payload, view.payload + 6 + length);
            if (out.size() >= 5) {
                std::uint8_t cnt = static_cast<std::uint8_t>(out[4] & kMbxCntMask);
                mailboxes_[static_cast<std::size_t>(slave)].mbx_cnt = cnt;
            }
            return true;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(kMbxRetryDelayMs));
    }
    set_error("mailbox read timeout");
    return false;
}

bool MuEthercat::eeprom_wait_not_busy(std::uint16_t aiadr, std::uint16_t& estat, int timeout_ms) {
    auto start = std::chrono::steady_clock::now();
    while (true) {
        std::vector<std::uint8_t> response;
        if (!master_->send_aprd(aiadr, kRegEepStat, 2, response, timeout_ms)) {
            set_error(master_->last_error().c_str());
            return false;
        }
        auto view = mu_ethercat::get_datagram_view(response.data(), response.size());
        if (view.payload == nullptr || view.payload_size < 2) {
            set_error("invalid eeprom status response");
            return false;
        }
        estat = read_u16_le(view.payload);
        if ((estat & kEstatBusy) == 0) {
            return true;
        }

        std::this_thread::sleep_for(std::chrono::microseconds(kEepromDelayUs));
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now() - start);
        if (elapsed.count() >= timeout_ms) {
            set_error("eeprom busy timeout");
            return false;
        }
    }
}

bool MuEthercat::eeprom_read_ap(std::uint16_t aiadr, std::uint16_t word_addr, std::uint32_t& out) {
    out = 0;
    std::uint16_t estat = 0;
    if (!eeprom_wait_not_busy(aiadr, estat, kEepromTimeoutMs)) {
        return false;
    }

    if (estat & kEstatEmask) {
        std::uint8_t cmd[2] = {0, 0};
        write_u16_le(cmd, kEcmdNop);
        std::vector<std::uint8_t> response;
        if (!master_->send_apwr(aiadr, kRegEepCtl, cmd, sizeof(cmd), response, kEepromTimeoutMs)) {
            set_error("clear eeprom error failed");
            return false;
        }
    }

    std::uint8_t ed[6] = {0};
    write_u16_le(ed, kEcmdRead);
    write_u16_le(ed + 2, word_addr);
    write_u16_le(ed + 4, 0x0000);

    std::vector<std::uint8_t> response;
    if (!master_->send_apwr(aiadr, kRegEepCtl, ed, sizeof(ed), response, kEepromTimeoutMs)) {
        set_error(master_->last_error().c_str());
        return false;
    }

    std::this_thread::sleep_for(std::chrono::microseconds(kEepromDelayUs));
    if (!eeprom_wait_not_busy(aiadr, estat, kEepromTimeoutMs)) {
        return false;
    }
    if (estat & kEstatNack) {
        set_error("eeprom nack");
        return false;
    }

    std::uint16_t read_len = (estat & kEstatR64) ? 8 : 4;
    if (!master_->send_aprd(aiadr, kRegEepDat, read_len, response, kEepromTimeoutMs)) {
        set_error(master_->last_error().c_str());
        return false;
    }
    auto view = mu_ethercat::get_datagram_view(response.data(), response.size());
    if (view.payload == nullptr || view.payload_size < 4) {
        set_error("invalid eeprom data response");
        return false;
    }

    out = static_cast<std::uint32_t>(read_u16_le(view.payload)) |
          (static_cast<std::uint32_t>(read_u16_le(view.payload + 2)) << 16);
    return true;
}
