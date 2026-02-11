#pragma once

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace mu_ethercat {

class Master {
 public:
    Master();
    ~Master();

    Master(const Master&) = delete;
    Master& operator=(const Master&) = delete;

    bool open(const char* ifname);
    void close();

    bool is_open() const;
    const std::string& last_error() const;

    bool send_frame(const std::uint8_t* data, std::size_t size);
    int recv_frame(std::uint8_t* out, std::size_t max_size, int timeout_ms);

    bool send_aprd(std::uint16_t adp,
                   std::uint16_t ado,
                   std::uint16_t length,
                   std::vector<std::uint8_t>& response,
                   int timeout_ms);
    bool send_apwr(std::uint16_t adp,
                   std::uint16_t ado,
                   const std::uint8_t* data,
                   std::size_t data_size,
                   std::vector<std::uint8_t>& response,
                   int timeout_ms);
    bool send_fpwr(std::uint16_t adp,
                   std::uint16_t ado,
                   const std::uint8_t* data,
                   std::size_t data_size,
                   std::vector<std::uint8_t>& response,
                   int timeout_ms);
    bool send_fprd(std::uint16_t adp,
                   std::uint16_t ado,
                   std::uint16_t length,
                   std::vector<std::uint8_t>& response,
                   int timeout_ms);
    bool send_brd(std::uint16_t adp,
                  std::uint16_t ado,
                  std::uint16_t length,
                  std::vector<std::uint8_t>& response,
                  int timeout_ms);
    bool send_bwr(std::uint16_t adp,
                  std::uint16_t ado,
                  const std::uint8_t* data,
                  std::size_t data_size,
                  std::vector<std::uint8_t>& response,
                  int timeout_ms);
    bool send_lrw(std::uint32_t logical_addr,
                  std::uint8_t* io_data,
                  std::size_t io_size,
                  std::vector<std::uint8_t>& response,
                  int timeout_ms);

 private:
    void set_error(const char* message);
    bool recv_expect_response(std::vector<std::uint8_t>& response, int timeout_ms);

    int sock_fd_ = -1;
    int if_index_ = -1;
    std::uint8_t if_mac_[6] = {0};
    bool has_mac_ = false;
    std::string last_error_;
};

}  // namespace mu_ethercat
