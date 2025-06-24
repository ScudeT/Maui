#ifndef WIRE_HPP
#define WIRE_HPP

#include <string>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <i2c/smbus.h>

/**
 * @brief Header-only I2C handler for Linux using /dev/i2c-* interface
 */
class I2CHandler {
public:
    /**
     * @brief Open specified I2C device (e.g. "/dev/i2c-1").
     * Exits on failure.
     */
    inline I2CHandler(const std::string& device) {
        i2c_fd = open(device.c_str(), O_RDWR);
        if (i2c_fd < 0) {
            perror("Failed to open the I2C bus");
            std::exit(EXIT_FAILURE);
        }
    }

    /**
     * @brief Default constructor opens "/dev/i2c-1".
     */
    inline I2CHandler() : I2CHandler(std::string("/dev/i2c-1")) {}

    /**
     * @brief Close I2C device on destruction.
     */
    inline ~I2CHandler() {
        if (i2c_fd >= 0) {
            ::close(i2c_fd);
        }
    }

    /**
     * @brief Write a single byte to a register.
     */
    inline void write_byte(uint8_t address, uint8_t subAddress, uint8_t data) {
        if (ioctl(i2c_fd, I2C_SLAVE, address) < 0) {
            perror("Failed to set I2C address");
            return;
        }
        uint8_t buffer[2] = {subAddress, data};
        if (::write(i2c_fd, buffer, 2) != 2) {
            perror("Failed to write to the I2C bus");
        }
    }

    /**
     * @brief Read a single byte from a register.
     */
    inline uint8_t read_byte(uint8_t address, uint8_t subAddress) {
        if (ioctl(i2c_fd, I2C_SLAVE, address) < 0) {
            perror("Failed to set I2C address");
            return 0;
        }
        if (::write(i2c_fd, &subAddress, 1) != 1) {
            perror("Failed to write to the I2C bus");
            return 0;
        }
        uint8_t data = 0;
        if (::read(i2c_fd, &data, 1) != 1) {
            perror("Failed to read from the I2C bus");
        }
        return data;
    }

    /**
     * @brief Read multiple bytes starting at a register.
     */
    inline void read_bytes(uint8_t address, uint8_t subAddress, uint8_t count, uint8_t* dest) {
        if (ioctl(i2c_fd, I2C_SLAVE, address) < 0) {
            perror("Failed to set I2C address");
            return;
        }
        if (::write(i2c_fd, &subAddress, 1) != 1) {
            perror("Failed to write to the I2C bus");
            return;
        }
        if (::read(i2c_fd, dest, count) != count) {
            perror("Failed to read from the I2C bus");
        }
    }

    /**
     * @brief Print an I2C error message to stderr.
     */
    inline void print_i2c_error(const char* error_message) {
        std::fprintf(stderr, "I2C ERROR: %s\n", error_message);
    }

private:
    int i2c_fd;  ///< File descriptor of the I2C device
};

#endif // WIRE_HPP
