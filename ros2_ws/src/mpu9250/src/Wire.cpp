#include <iostream>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <i2c/smbus.h>

#include "Wire.h"


I2CHandler::I2CHandler(const std::string& device) {
    // Open the I2C device
    i2c_fd = open(device.c_str(), O_RDWR);
    if (i2c_fd < 0) {
        perror("Failed to open the I2C bus");
        exit(1);
    }
}

I2CHandler::I2CHandler() {
    const std::string& device = "/dev/i2c-1";
    // Open the I2C device
    i2c_fd = open(device.c_str(), O_RDWR);
    if (i2c_fd < 0) {
        perror("Failed to open the I2C bus");
        exit(1);
    }
}

I2CHandler::~I2CHandler() {
    // Close the I2C device
    if (i2c_fd >= 0) close(i2c_fd);
}

void I2CHandler::write_byte(uint8_t address, uint8_t subAddress, uint8_t data) {
    if (ioctl(i2c_fd, I2C_SLAVE, address) < 0) {
        perror("Failed to set I2C address");
        return;
    }

    uint8_t buffer[2] = {subAddress, data};
    if (write(i2c_fd, buffer, 2) != 2) {
        perror("Failed to write to the I2C bus");
    }
}

uint8_t I2CHandler::read_byte(uint8_t address, uint8_t subAddress) {
    if (ioctl(i2c_fd, I2C_SLAVE, address) < 0) {
        perror("Failed to set I2C address");
        return 0;
    }

    // Write the subAddress to the device
    if (write(i2c_fd, &subAddress, 1) != 1) {
        perror("Failed to write to the I2C bus");
        return 0;
    }

    // Read a single byte from the device
    uint8_t data = 0;
    if (read(i2c_fd, &data, 1) != 1) {
        perror("Failed to read from the I2C bus");
    }
    return data;
}

void I2CHandler::read_bytes(uint8_t address, uint8_t subAddress, uint8_t count, uint8_t* dest) {
    if (ioctl(i2c_fd, I2C_SLAVE, address) < 0) {
        perror("Failed to set I2C address");
        return;
    }

    // Write the subAddress to the device
    if (write(i2c_fd, &subAddress, 1) != 1) {
        perror("Failed to write to the I2C bus");
        return;
    }

    // Read multiple bytes from the device
    if (read(i2c_fd, dest, count) != count) {
        perror("Failed to read from the I2C bus");
    }
}

void I2CHandler::print_i2c_error(const char* error_message) {
    std::cerr << "I2C ERROR: " << error_message << std::endl;
}
