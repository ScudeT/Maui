// pca9685.hpp
#ifndef PWM_PCA9685_HPP_
#define PWM_PCA9685_HPP_

#include <rclcpp/rclcpp.hpp>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <chrono>
#include <thread>
#include <cstring>

// Constants (registers etc.)
static constexpr uint8_t PCA9685_ADDRESS = 0x40;
static constexpr uint8_t PCA9685_MODE1_REG = 0x00;
static constexpr uint8_t PCA9685_MODE2_REG = 0x01;
static constexpr uint8_t PCA9685_PRESCALE_REG = 0xFE;
static constexpr uint8_t PCA9685_CHANNEL_0_REG = 0x06;

namespace pwm_pca9685 {

// Minimal SMBus definitions, reusing your constants from your last upload
static constexpr int I2C_SMBUS_WRITE = 0;
static constexpr int I2C_SMBUS_BLOCK_MAX = 32;

union i2c_smbus_data {
  __u8  byte;
  __u16 word;
  __u8  block[I2C_SMBUS_BLOCK_MAX+2];
};

struct i2c_smbus_ioctl_data {
  char read_write;
  __u8 command;
  int size;
  union i2c_smbus_data *data;
};

static inline __s32 _i2c_smbus_access(int file, char rw, __u8 cmd, int size, i2c_smbus_data *d) {
  i2c_smbus_ioctl_data args{rw, cmd, size, d};
  return ioctl(file, I2C_SMBUS, &args);
}

static inline __s32 i2c_write_byte_data(int file, __u8 cmd, __u8 val) {
  i2c_smbus_data d; d.byte = val;
  return _i2c_smbus_access(file, I2C_SMBUS_WRITE, cmd, 2, &d);
}

static inline __s32 i2c_write_i2c_block_data(int file, __u8 cmd, __u8 len, __u8 *vals) {
  i2c_smbus_data d;
  if (len > I2C_SMBUS_BLOCK_MAX) len = I2C_SMBUS_BLOCK_MAX;
  d.block[0] = len;
  for (int i = 0; i < len; ++i) d.block[i+1] = vals[i];
  return _i2c_smbus_access(file, I2C_SMBUS_WRITE, cmd, 8, &d);
}

class PCA9685 {
public:
  PCA9685(const rclcpp::Logger &logger,
          const std::string &device = "/dev/i2c-1",
          uint8_t address = PCA9685_ADDRESS,
          uint16_t frequency = 1600)
    : logger_(logger),
      device_(device),
      address_(address),
      frequency_(frequency),
      file_(-1)
  {}

  ~PCA9685() {
    if (file_ >= 0) {
      // Turn off all channels on destruction
      uint8_t off_block[4] = {0x00, 0x00, 0x00, 0x10};
      for (int ch = 0; ch < 16; ++ch) {
        i2c_write_i2c_block_data(file_, PCA9685_CHANNEL_0_REG + 4*ch, 4, off_block);
      }
      ::close(file_);
    }
  }

  // Open device, set slave address, and reset chip, etc.
  bool start() {
    file_ = ::open(device_.c_str(), O_RDWR);
    if (file_ < 0 || ioctl(file_, I2C_SLAVE, address_) < 0) {
      RCLCPP_ERROR(logger_, "Failed to open i2c device %s @0x%02x", device_.c_str(), address_);
      return false;
    }

    // Reset chip and set frequency (based on pca9685_activity::reset)
    if (!reset()) {
      RCLCPP_ERROR(logger_, "Failed to reset PCA9685 chip");
      return false;
    }

    return true;
  }

  // Set PWM duty cycle for one channel, with 16-bit input, converted to 12-bit and using full-on/off flags
  bool set(uint8_t channel, uint16_t value) {
    if (channel > 15) {
      RCLCPP_ERROR(logger_, "Channel %d out of range", channel);
      return false;
    }

    uint16_t value_12bit = value >> 4;  // convert 16-bit to 12-bit
    uint8_t values[4];

    if (value_12bit == 0x0FFF) {
      // Full ON
      values[0] = 0x00; // LEDn_ON_L
      values[1] = 0x10; // LEDn_ON_H: full on bit set
      values[2] = 0x00; // LEDn_OFF_L
      values[3] = 0x00; // LEDn_OFF_H
    } else if (value_12bit == 0x0000) {
      // Full OFF
      values[0] = 0x00; // LEDn_ON_L
      values[1] = 0x00; // LEDn_ON_H
      values[2] = 0x00; // LEDn_OFF_L
      values[3] = 0x10; // LEDn_OFF_H: full off bit set
    } else {
      // PWM mode
      uint16_t pwm_val = value_12bit + 1;
      values[0] = 0x00;                // LEDn_ON_L (start at 0)
      values[1] = 0x00;                // LEDn_ON_H
      values[2] = pwm_val & 0xFF;      // LEDn_OFF_L
      values[3] = (pwm_val >> 8) & 0x0F; // LEDn_OFF_H
    }

    int ret = i2c_write_i2c_block_data(file_, PCA9685_CHANNEL_0_REG + 4*channel, 4, values);
    if (ret < 0) {
      RCLCPP_ERROR(logger_, "Failed to write PWM data for channel %d", channel);
      return false;
    }
    return true;
  }

private:
  bool reset() {
    // Soft reset the chip and configure frequency, based on pca9685_activity::reset()

    // Reset mode1 register
    if (i2c_write_byte_data(file_, PCA9685_MODE1_REG, 0b10000000) < 0) {
      RCLCPP_ERROR(logger_, "Failed to reset MODE1 register");
      return false;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    // Calculate prescale
    uint8_t prescale = static_cast<uint8_t>(25000000.0 / 4096.0 / frequency_ + 0.5);

    // Sleep mode to set prescale
    if (i2c_write_byte_data(file_, PCA9685_MODE1_REG, 0b00010000) < 0) {
      RCLCPP_ERROR(logger_, "Failed to set sleep mode");
      return false;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(25));

    if (i2c_write_byte_data(file_, PCA9685_PRESCALE_REG, prescale) < 0) {
      RCLCPP_ERROR(logger_, "Failed to write prescale");
      return false;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(25));

    // MODE2: totem pole output
    if (i2c_write_byte_data(file_, PCA9685_MODE2_REG, 0x04) < 0) {
      RCLCPP_ERROR(logger_, "Failed to set MODE2");
      return false;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(25));

    // Wake up and auto-increment mode1
    if (i2c_write_byte_data(file_, PCA9685_MODE1_REG, 0xA1) < 0) {
      RCLCPP_ERROR(logger_, "Failed to wake up chip");
      return false;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(25));

    // Reset all PWM channels to off
    for (int ch = 0; ch < 16; ++ch) {
      uint8_t off_block[4] = {0x00, 0x00, 0x00, 0x10};
      i2c_write_i2c_block_data(file_, PCA9685_CHANNEL_0_REG + 4*ch, 4, off_block);
    }

    RCLCPP_INFO(logger_, "PCA9685 initialized @ 0x%02X frequency %d Hz", address_, frequency_);
    return true;
  }

  rclcpp::Logger logger_;
  std::string device_;
  uint8_t address_;
  uint16_t frequency_;
  int file_;
};

}  // namespace pwm_pca9685

#endif  // PWM_PCA9685_HPP_
