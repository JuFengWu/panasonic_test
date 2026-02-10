#include <array>
#include <cstdint>
#include <string>

enum class EthercatState : std::uint16_t {
    Init = 0x01,
    PreOp = 0x02,
    SafeOp = 0x04,
    Operational = 0x08
};

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
    virtual int config_pdo_mapping(void* io_map) = 0;
    virtual bool setting_dc() = 0;
    virtual bool read_ehtercat_state() = 0;
    virtual bool write_ethercat_state(std::uint16_t slave) = 0;
    virtual int state_check(std::uint16_t slave, std::uint16_t state, int timeout) = 0;
    virtual bool set_pdo_data() = 0;
    virtual bool get_pdo_data() = 0;
    virtual bool close_ethercat() = 0;

    virtual int get_slave_count() const = 0;
    virtual bool set_slave_state(std::uint16_t slave, std::uint16_t state) = 0;
    virtual std::uint16_t get_slave_state(std::uint16_t slave) const = 0;
    virtual std::uint16_t get_slave_al_status(std::uint16_t slave) const = 0;
    virtual std::uint8_t* get_slave_outputs(std::uint16_t slave) = 0;
    virtual std::uint8_t* get_slave_inputs(std::uint16_t slave) = 0;
    virtual int get_slave_obytes(std::uint16_t slave) const = 0;
    virtual int get_slave_ibytes(std::uint16_t slave) const = 0;

    virtual int timeout_state() const = 0;
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

 private:
    std::string last_error_;
};
    
