#include <array>
#include <cstdint>
#include <string>

class MyEthercat {
 public:
    virtual ~MyEthercat() = default;

    virtual bool init(const char* ifname) = 0;
    virtual void close() = 0;
    virtual bool read_sdo_state(std::uint16_t slave,
                                std::uint16_t index,
                                std::uint8_t sub,
                                void* out,
                                int size) = 0;
    virtual bool write_sdo_state(std::uint16_t slave,
                                 std::uint16_t index,
                                 std::uint8_t sub,
                                 const void* data,
                                 int size) = 0;
    virtual std::string get_error_message() const = 0;
    virtual bool scan_slaves() = 0;
    virtual bool setting_pdo_mapping(int slave_id) = 0;
    virtual bool setting_dc() = 0;
    virtual bool read_ehtercat_state() = 0;
    virtual bool write_ethercat_state() = 0;
    virtual bool check_ethercat_state() = 0;
    virtual bool set_pdo_data() = 0;
    virtual bool get_pdo_data() = 0;
    virtual bool close_ethercat() = 0;
};

class SoemEthercat : public MyEthercat {
 public:
    SoemEthercat() = default;
    ~SoemEthercat() override = default;

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
    bool setting_pdo_mapping(int slave_id) override;
    bool setting_dc() override;
    bool read_ehtercat_state() override;
    bool write_ethercat_state() override;
    bool check_ethercat_state() override;
    bool set_pdo_data() override;
    bool get_pdo_data() override;
    bool close_ethercat() override;

 private:
    std::string last_error_;
    std::array<char, 4096> io_map_{};
};
    
