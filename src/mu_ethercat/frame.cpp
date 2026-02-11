#include "mu_ethercat/frame.hpp"

#include <algorithm>
#include <cstring>

namespace mu_ethercat {

namespace {
constexpr std::uint16_t kEtherType = 0x88A4;
constexpr std::size_t kEthHeaderSize = 14;
constexpr std::size_t kEcatHeaderSize = 2;
constexpr std::size_t kDatagramHeaderSize = 10;
constexpr std::size_t kWkcSize = 2;

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

std::vector<std::uint8_t> build_single_datagram(DatagramCmd cmd,
                                                std::uint16_t adp,
                                                std::uint16_t ado,
                                                std::uint16_t length,
                                                std::uint16_t irq,
                                                const std::uint8_t* data,
                                                std::size_t data_size) {
    std::size_t payload_size = length;
    std::size_t frame_size = kEthHeaderSize + kEcatHeaderSize + kDatagramHeaderSize + payload_size + kWkcSize;

    std::vector<std::uint8_t> frame(frame_size, 0);
    std::uint8_t* ptr = frame.data();

    // Ethernet header is left zeroed; caller can fill dst/src MAC.
    write_u16_le(ptr + 12, kEtherType);

    std::uint8_t* ecat = ptr + kEthHeaderSize;
    std::uint16_t ecat_len = static_cast<std::uint16_t>(kDatagramHeaderSize + payload_size + kWkcSize);
    write_u16_le(ecat, ecat_len);

    std::uint8_t* dg = ecat + kEcatHeaderSize;
    dg[0] = static_cast<std::uint8_t>(cmd);
    dg[1] = 0x00;  // index
    write_u16_le(dg + 2, adp);
    write_u16_le(dg + 4, ado);
    write_u16_le(dg + 6, length);
    write_u16_le(dg + 8, irq);

    if (payload_size > 0 && data != nullptr) {
        std::size_t copy_size = std::min<std::size_t>(data_size, payload_size);
        std::memcpy(dg + kDatagramHeaderSize, data, copy_size);
    }

    return frame;
}

std::vector<std::uint8_t> build_single_logical(DatagramCmd cmd,
                                               std::uint32_t logical_addr,
                                               std::uint16_t length,
                                               std::uint16_t irq,
                                               const std::uint8_t* data,
                                               std::size_t data_size) {
    std::size_t payload_size = length;
    std::size_t frame_size = kEthHeaderSize + kEcatHeaderSize + kDatagramHeaderSize + payload_size + kWkcSize;

    std::vector<std::uint8_t> frame(frame_size, 0);
    std::uint8_t* ptr = frame.data();

    write_u16_le(ptr + 12, kEtherType);

    std::uint8_t* ecat = ptr + kEthHeaderSize;
    std::uint16_t ecat_len = static_cast<std::uint16_t>(kDatagramHeaderSize + payload_size + kWkcSize);
    write_u16_le(ecat, ecat_len);

    std::uint8_t* dg = ecat + kEcatHeaderSize;
    dg[0] = static_cast<std::uint8_t>(cmd);
    dg[1] = 0x00;
    write_u32_le(dg + 2, logical_addr);
    write_u16_le(dg + 6, length);
    write_u16_le(dg + 8, irq);

    if (payload_size > 0 && data != nullptr) {
        std::size_t copy_size = std::min<std::size_t>(data_size, payload_size);
        std::memcpy(dg + kDatagramHeaderSize, data, copy_size);
    }

    return frame;
}
}

std::vector<std::uint8_t> build_aprd(std::uint16_t adp,
                                     std::uint16_t ado,
                                     std::uint16_t length,
                                     std::uint16_t irq,
                                     const std::uint8_t* data,
                                     std::size_t data_size) {
    return build_single_datagram(DatagramCmd::APRD, adp, ado, length, irq, data, data_size);
}

std::vector<std::uint8_t> build_apwr(std::uint16_t adp,
                                     std::uint16_t ado,
                                     std::uint16_t length,
                                     std::uint16_t irq,
                                     const std::uint8_t* data,
                                     std::size_t data_size) {
    return build_single_datagram(DatagramCmd::APWR, adp, ado, length, irq, data, data_size);
}

std::vector<std::uint8_t> build_fpwr(std::uint16_t adp,
                                     std::uint16_t ado,
                                     std::uint16_t length,
                                     std::uint16_t irq,
                                     const std::uint8_t* data,
                                     std::size_t data_size) {
    return build_single_datagram(DatagramCmd::FPWR, adp, ado, length, irq, data, data_size);
}

std::vector<std::uint8_t> build_fprd(std::uint16_t adp,
                                     std::uint16_t ado,
                                     std::uint16_t length,
                                     std::uint16_t irq,
                                     const std::uint8_t* data,
                                     std::size_t data_size) {
    return build_single_datagram(DatagramCmd::FPRD, adp, ado, length, irq, data, data_size);
}

std::vector<std::uint8_t> build_brd(std::uint16_t adp,
                                    std::uint16_t ado,
                                    std::uint16_t length,
                                    std::uint16_t irq,
                                    const std::uint8_t* data,
                                    std::size_t data_size) {
    return build_single_datagram(DatagramCmd::BRD, adp, ado, length, irq, data, data_size);
}

std::vector<std::uint8_t> build_bwr(std::uint16_t adp,
                                    std::uint16_t ado,
                                    std::uint16_t length,
                                    std::uint16_t irq,
                                    const std::uint8_t* data,
                                    std::size_t data_size) {
    return build_single_datagram(DatagramCmd::BWR, adp, ado, length, irq, data, data_size);
}

std::vector<std::uint8_t> build_lrw(std::uint32_t logical_addr,
                                    std::uint16_t length,
                                    std::uint16_t irq,
                                    const std::uint8_t* data,
                                    std::size_t data_size) {
    return build_single_logical(DatagramCmd::LRW, logical_addr, length, irq, data, data_size);
}

DatagramView get_datagram_view(std::uint8_t* frame, std::size_t size) {
    DatagramView view{};
    if (frame == nullptr || size < (kEthHeaderSize + kEcatHeaderSize + kDatagramHeaderSize + kWkcSize)) {
        return view;
    }

    std::uint8_t* dg = frame + kEthHeaderSize + kEcatHeaderSize;
    view.data = dg;
    view.size = size - (kEthHeaderSize + kEcatHeaderSize);

    std::uint16_t length = static_cast<std::uint16_t>(dg[6] | (dg[7] << 8));
    std::size_t payload_size = length;
    std::size_t total = kDatagramHeaderSize + payload_size + kWkcSize;
    if (total > view.size) {
        return DatagramView{};
    }

    view.payload = dg + kDatagramHeaderSize;
    view.payload_size = payload_size;
    view.wkc = reinterpret_cast<std::uint16_t*>(dg + kDatagramHeaderSize + payload_size);
    return view;
}

}  // namespace mu_ethercat
