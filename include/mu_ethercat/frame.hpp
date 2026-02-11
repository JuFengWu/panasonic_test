#pragma once

#include <cstddef>
#include <cstdint>
#include <vector>

namespace mu_ethercat {

enum class DatagramCmd : std::uint8_t {
    APRD = 0x01,
    APWR = 0x02,
    APRW = 0x03,
    FPRD = 0x04,
    FPWR = 0x05,
    FPRW = 0x06,
    BRD = 0x07,
    BWR = 0x08,
    BRW = 0x09,
    LRD = 0x0A,
    LWR = 0x0B,
    LRW = 0x0C
};

struct DatagramView {
    std::uint8_t* data = nullptr;
    std::size_t size = 0;
    std::uint8_t* payload = nullptr;
    std::size_t payload_size = 0;
    std::uint16_t* wkc = nullptr;
};

std::vector<std::uint8_t> build_aprd(std::uint16_t adp,
                                     std::uint16_t ado,
                                     std::uint16_t length,
                                     std::uint16_t irq,
                                     const std::uint8_t* data,
                                     std::size_t data_size);

std::vector<std::uint8_t> build_apwr(std::uint16_t adp,
                                     std::uint16_t ado,
                                     std::uint16_t length,
                                     std::uint16_t irq,
                                     const std::uint8_t* data,
                                     std::size_t data_size);

std::vector<std::uint8_t> build_fpwr(std::uint16_t adp,
                                     std::uint16_t ado,
                                     std::uint16_t length,
                                     std::uint16_t irq,
                                     const std::uint8_t* data,
                                     std::size_t data_size);

std::vector<std::uint8_t> build_fprd(std::uint16_t adp,
                                     std::uint16_t ado,
                                     std::uint16_t length,
                                     std::uint16_t irq,
                                     const std::uint8_t* data,
                                     std::size_t data_size);

std::vector<std::uint8_t> build_brd(std::uint16_t adp,
                                    std::uint16_t ado,
                                    std::uint16_t length,
                                    std::uint16_t irq,
                                    const std::uint8_t* data,
                                    std::size_t data_size);

std::vector<std::uint8_t> build_bwr(std::uint16_t adp,
                                    std::uint16_t ado,
                                    std::uint16_t length,
                                    std::uint16_t irq,
                                    const std::uint8_t* data,
                                    std::size_t data_size);

std::vector<std::uint8_t> build_lrw(std::uint32_t logical_addr,
                                    std::uint16_t length,
                                    std::uint16_t irq,
                                    const std::uint8_t* data,
                                    std::size_t data_size);

DatagramView get_datagram_view(std::uint8_t* frame, std::size_t size);

}  // namespace mu_ethercat
