#include "mu_ethercat/master.hpp"

#include "mu_ethercat/frame.hpp"

#ifdef __linux__
#include <arpa/inet.h>
#include <cerrno>
#include <cstring>
#include <net/ethernet.h>
#include <linux/if_ether.h>
#include <net/if.h>
#include <netpacket/packet.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>
#endif

#ifndef ETH_P_ETHERCAT
#define ETH_P_ETHERCAT 0x88A4
#endif

namespace mu_ethercat {

Master::Master() = default;

Master::~Master() {
    close();
}

void Master::set_error(const char* message) {
    last_error_.assign(message ? message : "unknown error");
}

bool Master::open(const char* ifname) {
    last_error_.clear();
#ifdef __linux__
    if (ifname == nullptr || *ifname == '\0') {
        set_error("ifname is null or empty");
        return false;
    }

    if (sock_fd_ >= 0) {
        return true;
    }

    sock_fd_ = ::socket(PF_PACKET, SOCK_RAW, htons(ETH_P_ETHERCAT));
    if (sock_fd_ < 0) {
        set_error("socket(PF_PACKET) failed");
        return false;
    }

    struct ifreq ifr {};
    std::strncpy(ifr.ifr_name, ifname, sizeof(ifr.ifr_name) - 1);
    if (ioctl(sock_fd_, SIOCGIFINDEX, &ifr) < 0) {
        set_error("SIOCGIFINDEX failed");
        close();
        return false;
    }
    if_index_ = ifr.ifr_ifindex;

    struct sockaddr_ll sll {};
    sll.sll_family = AF_PACKET;
    sll.sll_ifindex = if_index_;
    sll.sll_protocol = htons(ETH_P_ETHERCAT);
    if (bind(sock_fd_, reinterpret_cast<struct sockaddr*>(&sll), sizeof(sll)) < 0) {
        set_error("bind(AF_PACKET) failed");
        close();
        return false;
    }

    return true;
#else
    (void)ifname;
    set_error("mu_ethercat master only supports Linux");
    return false;
#endif
}

void Master::close() {
#ifdef __linux__
    if (sock_fd_ >= 0) {
        ::close(sock_fd_);
        sock_fd_ = -1;
    }
    if_index_ = -1;
#endif
}

bool Master::is_open() const {
    return sock_fd_ >= 0;
}

const std::string& Master::last_error() const {
    return last_error_;
}

bool Master::send_frame(const std::uint8_t* data, std::size_t size) {
    last_error_.clear();
#ifdef __linux__
    if (sock_fd_ < 0) {
        set_error("socket not open");
        return false;
    }
    if (data == nullptr || size == 0) {
        set_error("invalid frame");
        return false;
    }

    ssize_t sent = ::send(sock_fd_, data, size, 0);
    if (sent < 0 || static_cast<std::size_t>(sent) != size) {
        set_error("send failed");
        return false;
    }
    return true;
#else
    (void)data;
    (void)size;
    set_error("mu_ethercat master only supports Linux");
    return false;
#endif
}

int Master::recv_frame(std::uint8_t* out, std::size_t max_size, int timeout_ms) {
    last_error_.clear();
#ifdef __linux__
    if (sock_fd_ < 0) {
        set_error("socket not open");
        return -1;
    }
    if (out == nullptr || max_size == 0) {
        set_error("invalid output buffer");
        return -1;
    }

    if (timeout_ms >= 0) {
        struct timeval tv {};
        tv.tv_sec = timeout_ms / 1000;
        tv.tv_usec = (timeout_ms % 1000) * 1000;
        if (setsockopt(sock_fd_, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv)) < 0) {
            set_error("setsockopt(SO_RCVTIMEO) failed");
            return -1;
        }
    }

    ssize_t received = ::recv(sock_fd_, out, max_size, 0);
    if (received < 0) {
        if (errno == EAGAIN || errno == EWOULDBLOCK) {
            return 0;
        }
        set_error("recv failed");
        return -1;
    }
    return static_cast<int>(received);
#else
    (void)out;
    (void)max_size;
    (void)timeout_ms;
    set_error("mu_ethercat master only supports Linux");
    return -1;
#endif
}

bool Master::recv_expect_response(std::vector<std::uint8_t>& response, int timeout_ms) {
    response.clear();
    std::vector<std::uint8_t> buffer(2048);
    int received = recv_frame(buffer.data(), buffer.size(), timeout_ms);
    if (received <= 0) {
        if (received == 0) {
            set_error("timeout waiting for response");
        }
        return false;
    }
    buffer.resize(static_cast<std::size_t>(received));
    response = buffer;
    return true;
}

bool Master::send_aprd(std::uint16_t adp,
                       std::uint16_t ado,
                       std::uint16_t length,
                       std::vector<std::uint8_t>& response,
                       int timeout_ms) {
    auto frame = build_aprd(adp, ado, length, 0, nullptr, 0);
    if (!send_frame(frame.data(), frame.size())) {
        return false;
    }
    return recv_expect_response(response, timeout_ms);
}

bool Master::send_apwr(std::uint16_t adp,
                       std::uint16_t ado,
                       const std::uint8_t* data,
                       std::size_t data_size,
                       std::vector<std::uint8_t>& response,
                       int timeout_ms) {
    auto frame = build_apwr(adp, ado, static_cast<std::uint16_t>(data_size), 0, data, data_size);
    if (!send_frame(frame.data(), frame.size())) {
        return false;
    }
    return recv_expect_response(response, timeout_ms);
}

bool Master::send_fpwr(std::uint16_t adp,
                       std::uint16_t ado,
                       const std::uint8_t* data,
                       std::size_t data_size,
                       std::vector<std::uint8_t>& response,
                       int timeout_ms) {
    auto frame = build_fpwr(adp, ado, static_cast<std::uint16_t>(data_size), 0, data, data_size);
    if (!send_frame(frame.data(), frame.size())) {
        return false;
    }
    return recv_expect_response(response, timeout_ms);
}

bool Master::send_fprd(std::uint16_t adp,
                       std::uint16_t ado,
                       std::uint16_t length,
                       std::vector<std::uint8_t>& response,
                       int timeout_ms) {
    auto frame = build_fprd(adp, ado, length, 0, nullptr, 0);
    if (!send_frame(frame.data(), frame.size())) {
        return false;
    }
    return recv_expect_response(response, timeout_ms);
}

bool Master::send_brd(std::uint16_t adp,
                      std::uint16_t ado,
                      std::uint16_t length,
                      std::vector<std::uint8_t>& response,
                      int timeout_ms) {
    auto frame = build_brd(adp, ado, length, 0, nullptr, 0);
    if (!send_frame(frame.data(), frame.size())) {
        return false;
    }
    return recv_expect_response(response, timeout_ms);
}

bool Master::send_bwr(std::uint16_t adp,
                      std::uint16_t ado,
                      const std::uint8_t* data,
                      std::size_t data_size,
                      std::vector<std::uint8_t>& response,
                      int timeout_ms) {
    auto frame = build_bwr(adp, ado, static_cast<std::uint16_t>(data_size), 0, data, data_size);
    if (!send_frame(frame.data(), frame.size())) {
        return false;
    }
    return recv_expect_response(response, timeout_ms);
}

bool Master::send_lrw(std::uint32_t logical_addr,
                      std::uint8_t* io_data,
                      std::size_t io_size,
                      std::vector<std::uint8_t>& response,
                      int timeout_ms) {
    auto frame = build_lrw(logical_addr, static_cast<std::uint16_t>(io_size), 0, io_data, io_size);
    if (!send_frame(frame.data(), frame.size())) {
        return false;
    }
    return recv_expect_response(response, timeout_ms);
}

}  // namespace mu_ethercat
