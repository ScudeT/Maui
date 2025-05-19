// pca9685.hpp
#ifndef PWM_PCA9685_HPP_
#define PWM_PCA9685_HPP_

#include <rclcpp/rclcpp.hpp>
#include <chrono>
#include <thread>
#include <cstring>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>

namespace pwm_pca9685 {

static constexpr uint8_t DEFAULT_ADDRESS = 0x40;
static constexpr uint8_t MODE1       = 0x00;
static constexpr uint8_t MODE2       = 0x01;
static constexpr uint8_t PRESCALE    = 0xFE;
static constexpr uint8_t LED0_ON_L   = 0x06;

static constexpr int MAX_RETRIES        = 3;
static constexpr int RETRY_DELAY_MS     = 10;
static constexpr double OSC_CLOCK_HZ    = 25e6;
static constexpr uint16_t PWM_RESOLUTION = 4096;

// --- minimal SMBus routines (from smbus_functions.h) ---
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
  return _i2c_smbus_access(file, I2C_SMBUS_WRITE, cmd, I2C_SMBUS_BYTE_DATA, &d);
}
static inline __s32 i2c_write_i2c_block_data(int file, __u8 cmd, __u8 len, __u8 *vals) {
  i2c_smbus_data d;
  if (len>32) len=32;
  for(int i=1;i<=len;i++) d.block[i]=vals[i-1];
  d.block[0]=len;
  return _i2c_smbus_access(file, I2C_SMBUS_WRITE, cmd, I2C_SMBUS_I2C_BLOCK_DATA, &d);
}
// ---------------------------------------------------------

class PCA9685 {
public:
  PCA9685(const rclcpp::Logger &logger,
          const std::string &device = "/dev/i2c-1",
          uint8_t address = DEFAULT_ADDRESS,
          uint16_t freq_hz = 1000)
  : logger_(logger),
    device_(device),
    address_(address),
    freq_hz_(freq_hz),
    file_(-1)
  {}

  ~PCA9685() {
    if (file_>=0) {
      // set all channels off on destruction
      uint8_t off_block[4] = {0x00, 0x00, 0x00, 0x10};
      for(int ch=0;ch<16;ch++){
        writeBlock(LED0_ON_L + 4*ch, off_block, 4);
      }
      close(file_);
    }
  }

  bool begin() {
    file_ = ::open(device_.c_str(), O_RDWR);
    if (file_<0 || ioctl(file_, I2C_SLAVE, address_)<0) {
      RCLCPP_ERROR(logger_, "Failed to open I2C device '%s' @0x%02x", device_.c_str(), address_);
      return false;
    }
    return reset();
  }

  bool setDutyCycle(uint8_t channel, uint16_t value12) {
    if (channel>15) {
      RCLCPP_ERROR(logger_, "Channel %u out of range [0..15]", channel);
      return false;
    }
    if (value12>=PWM_RESOLUTION) value12 = PWM_RESOLUTION-1;
    uint16_t on  = 0;
    uint16_t off = value12;
    return setPWM(channel, on, off);
  }

  bool setPWM(uint8_t channel, uint16_t on, uint16_t off) {
    uint8_t buf[4] = {
      static_cast<uint8_t>(on & 0xFF),
      static_cast<uint8_t>((on>>8)&0x0F),
      static_cast<uint8_t>(off & 0xFF),
      static_cast<uint8_t>((off>>8)&0x0F)
    };
    return writeBlock(LED0_ON_L + 4*channel, buf, 4);
  }

private:
  const rclcpp::Logger logger_;
  const std::string device_;
  const uint8_t address_;
  const uint16_t freq_hz_;
  int file_;

  bool writeByte(uint8_t reg, uint8_t val) {
    for (int i=0; i<MAX_RETRIES; ++i) {
      if (i2c_write_byte_data(file_, reg, val)>=0) return true;
      RCLCPP_WARN(logger_, "I2C write reg 0x%02x failed, retry %d/%d", reg, i+1, MAX_RETRIES);
      std::this_thread::sleep_for(std::chrono::milliseconds(RETRY_DELAY_MS));
    }
    RCLCPP_ERROR(logger_, "I2C write reg 0x%02x persistent failure", reg);
    return false;
  }

  bool writeBlock(uint8_t reg, uint8_t *buf, size_t len) {
    for (int i=0; i<MAX_RETRIES; ++i) {
      if (i2c_write_i2c_block_data(file_, reg, len, buf)>=0) return true;
      RCLCPP_WARN(logger_, "I2C block write reg 0x%02x len %zu failed, retry %d/%d", reg, len, i+1, MAX_RETRIES);
      std::this_thread::sleep_for(std::chrono::milliseconds(RETRY_DELAY_MS));
    }
    RCLCPP_ERROR(logger_, "I2C block write reg 0x%02x len %zu persistent failure", reg, len);
    return false;
  }

  bool reset() {
    // go to sleep to set prescale
    if (!writeByte(MODE1, 0x10)) return false;
    std::this_thread::sleep_for(std::chrono::milliseconds(5));

    // calculate prescale
    double prescale_val = OSC_CLOCK_HZ / (PWM_RESOLUTION * freq_hz_) - 1.0;
    uint8_t prescale = static_cast<uint8_t>(prescale_val + 0.5);

    if (!writeByte(PRESCALE, prescale)) return false;

    // wake up, auto-increment
    if (!writeByte(MODE1, 0x20)) return false;
    // set output driver to totem-pole
    if (!writeByte(MODE2, 0x04)) return false;
    std::this_thread::sleep_for(std::chrono::milliseconds(5));

    RCLCPP_INFO(logger_, "PCA9685 @0x%02x initialized at %u Hz", address_, freq_hz_);
    return true;
  }
};

}  // namespace pwm_pca9685

#endif  // PWM_PCA9685_HPP_
