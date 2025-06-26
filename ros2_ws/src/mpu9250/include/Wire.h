#include <cmath>
#include <cstdint>
#include <chrono>
#include <string>

class I2CHandler {
public:
    I2CHandler(const std::string& device);
    I2CHandler();
    ~I2CHandler();

    void write_byte(uint8_t address, uint8_t subAddress, uint8_t data);

    uint8_t read_byte(uint8_t address, uint8_t subAddress);
    void read_bytes(uint8_t address, uint8_t subAddress, uint8_t count, uint8_t* dest);

    void print_i2c_error(const char* error_message);

private:
    int i2c_fd;  // File descriptor for the I2C device
};


