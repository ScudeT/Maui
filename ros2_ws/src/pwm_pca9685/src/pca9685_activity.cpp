#include "pwm_pca9685/pca9685_activity.h"
#include <chrono>
#include <thread>

using namespace std::chrono_literals;

namespace pwm_pca9685 {

PCA9685Activity::PCA9685Activity(rclcpp::Node::SharedPtr node) : node_(node) {
    RCLCPP_INFO(node_->get_logger(), "initializing");

    // declare and get parameters
    param_device = node_->declare_parameter("device", std::string("/dev/i2c-1"));
    param_address = node_->declare_parameter("address", PCA9685_ADDRESS);
    param_frequency = node_->declare_parameter("frequency", 1600);

    std::vector<int64_t> default_timeout = {5000, 5000, 5000, 5000, 5000, 5000, 5000, 5000,
        5000, 5000, 5000, 5000, 5000, 5000, 5000, 5000};

    param_timeout = node_->declare_parameter<std::vector<int64_t>>("timeout", default_timeout);

    param_pwm_min = node_->declare_parameter<std::vector<int64_t>>("pwm_min", std::vector<int64_t>{
        0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0
    });

    param_pwm_max = node_->declare_parameter<std::vector<int64_t>>("pwm_max", std::vector<int64_t>{
        65535, 65535, 65535, 65535, 65535, 65535, 65535, 65535,
        65535, 65535, 65535, 65535, 65535, 65535, 65535, 65535
    });

    param_timeout_value = node_->declare_parameter<std::vector<int64_t>>("timeout_value", std::vector<int64_t>{
        0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0
    });

    if(param_timeout.size() != 16) {
        RCLCPP_ERROR(node_->get_logger(), "size of param timeout must be 16");
        rclcpp::shutdown();
    }

    if(param_timeout_value.size() != 16) {
        RCLCPP_ERROR(node_->get_logger(), "size of param timeout_value must be 16");
        rclcpp::shutdown();
    }

    if(param_pwm_min.size() != 16) {
        RCLCPP_ERROR(node_->get_logger(), "size of param pwm_min must be 16");
        rclcpp::shutdown();
    }

    if(param_pwm_max.size() != 16) {
        RCLCPP_ERROR(node_->get_logger(), "size of param pwm_max must be 16");
        rclcpp::shutdown();
    }

    if(param_address < 0 || param_address > 127) {
        RCLCPP_ERROR(node_->get_logger(), "param address must be between 0 and 127 inclusive");
        rclcpp::shutdown();
    }

    if(param_frequency <= 0) {
        RCLCPP_ERROR(node_->get_logger(), "param frequency must be positive");
        rclcpp::shutdown();
    }

    // Initialize timing arrays using the current time in milliseconds.
    auto now = node_->get_clock()->now();
    uint64_t t = now.nanoseconds() / 1000000;
    for (int channel = 0; channel < 16; channel++) {
        last_set_times[channel] = t;
        last_change_times[channel] = t;
        last_data[channel] = 0;
    }
}

bool PCA9685Activity::reset() {
    // Reset the chip
    _i2c_smbus_write_byte_data(file, PCA9685_MODE1_REG, 0b10000000);
    std::this_thread::sleep_for(500ms);

    // Set frequency
    uint8_t prescale = static_cast<uint8_t>(25000000.0 / 4096.0 / param_frequency + 0.5);

    _i2c_smbus_write_byte_data(file, PCA9685_MODE1_REG, 0b00010000); // sleep
    std::this_thread::sleep_for(25ms);

    _i2c_smbus_write_byte_data(file, PCA9685_PRESCALE_REG, prescale); // set prescale
    std::this_thread::sleep_for(25ms);

    _i2c_smbus_write_byte_data(file, PCA9685_MODE2_REG, 0x04); // outdrv
    std::this_thread::sleep_for(25ms);

    _i2c_smbus_write_byte_data(file, PCA9685_MODE1_REG, 0xA1); // wake up
    std::this_thread::sleep_for(25ms);

    _i2c_smbus_write_byte_data(file, PCA9685_CHANNEL_ALL_REG, 0);
    _i2c_smbus_write_byte_data(file, PCA9685_CHANNEL_ALL_REG + 1, 0);
    _i2c_smbus_write_byte_data(file, PCA9685_CHANNEL_ALL_REG + 2, 0);
    _i2c_smbus_write_byte_data(file, PCA9685_CHANNEL_ALL_REG + 3, 0);

    return true;
}

/*
bool PCA9685Activity::set(uint8_t channel, uint16_t value) {
    uint16_t value_12bit = value >> 4;
    uint8_t values[4];
    if (value_12bit == 0x0FFF) {  // always on
        values[0] = 0x10;
        values[1] = 0x00;
        values[2] = 0x00;
        values[3] = 0x00;
    } else if (value_12bit == 0x0000) {  // always off
        values[0] = 0x00;
        values[1] = 0x00;
        values[2] = 0x10;
        values[3] = 0x00;
    } else {  // PWM
        values[0] = 0x00;
        values[1] = 0x00;
        values[2] = (value_12bit + 1) & 0xFF;
        values[3] = (value_12bit + 1) >> 8;
    }

    _i2c_smbus_write_i2c_block_data(file, PCA9685_CHANNEL_0_REG + (channel * 4), 4, values);
    return true;
} */

bool PCA9685Activity::set(uint8_t channel, uint16_t value) {
    // Shift the 16-bit value to 12-bit resolution.
    uint16_t value_12bit = value >> 4;
    uint8_t values[4];

    // For full on, set the full on flag in LEDn_ON_H (bit 4)
    if (value_12bit == 0x0FFF) {
        values[0] = 0x00;      // LEDn_ON_L
        values[1] = 0x10;      // LEDn_ON_H: full on flag
        values[2] = 0x00;      // LEDn_OFF_L
        values[3] = 0x00;      // LEDn_OFF_H
    }
    // For full off, set the full off flag in LEDn_OFF_H (bit 4)
    else if (value_12bit == 0x0000) {
        values[0] = 0x00;      // LEDn_ON_L
        values[1] = 0x00;      // LEDn_ON_H
        values[2] = 0x00;      // LEDn_OFF_L
        values[3] = 0x10;      // LEDn_OFF_H: full off flag
    }
    // Otherwise, compute the PWM on/off timings.
    else {
        uint16_t pwm_val = value_12bit + 1;
        values[0] = 0x00;             // LEDn_ON_L: start at 0
        values[1] = 0x00;             // LEDn_ON_H
        values[2] = pwm_val & 0xFF;     // LEDn_OFF_L: lower 8 bits
        values[3] = (pwm_val >> 8) & 0x0F;  // LEDn_OFF_H: upper 4 bits (bit4 remains clear)
    }

    _i2c_smbus_write_i2c_block_data(file, PCA9685_CHANNEL_0_REG + (channel * 4), 4, values);
    return true;
}

bool PCA9685Activity::start() {
    RCLCPP_INFO(node_->get_logger(), "starting");

    if (!sub_command) {
        sub_command = node_->create_subscription<std_msgs::msg::Int32MultiArray>(
            "command", 10,
            std::bind(&PCA9685Activity::onCommand, this, std::placeholders::_1));
    }

    file = open(param_device.c_str(), O_RDWR);
    if (file < 0 || ioctl(file, I2C_SLAVE, param_address) < 0) {
        RCLCPP_ERROR(node_->get_logger(), "i2c device open failed");
        return false;
    }

    if (!reset()) {
        RCLCPP_ERROR(node_->get_logger(), "chip reset and setup failed");
        return false;
    }

    return true;
}

bool PCA9685Activity::spinOnce() {
    auto now = node_->get_clock()->now();
    uint64_t t = now.nanoseconds() / 1000000;
    if (seq++ % 10 == 0) {
        for (int channel = 0; channel < 16; channel++) {
            if (param_timeout[channel] > 0 && t - last_set_times[channel] > static_cast<uint64_t>(std::abs(param_timeout[channel]))) {
                set(channel, param_timeout_value[channel]);
            } else if (param_timeout[channel] < 0 && t - last_change_times[channel] > static_cast<uint64_t>(std::abs(param_timeout[channel]))) {
                set(channel, param_timeout_value[channel]);
                RCLCPP_WARN(node_->get_logger(), "timeout %d", channel);
            }
        }
    }
    return true;
}

bool PCA9685Activity::stop() {
    RCLCPP_INFO(node_->get_logger(), "stopping");
    for (int channel = 0; channel < 16; channel++) {
        set(channel, param_timeout_value[channel]);
    }
    sub_command.reset();
    return true;
}

void PCA9685Activity::onCommand(const std_msgs::msg::Int32MultiArray::SharedPtr msg) {
    auto now = node_->get_clock()->now();
    uint64_t t = now.nanoseconds() / 1000000;
    if (msg->data.size() != 16) {
        RCLCPP_ERROR(node_->get_logger(), "array does not have a size of 16");
        return;
    }

    for (int channel = 0; channel < 16; channel++) {
        if (msg->data[channel] < 0)
            continue;

        if (msg->data[channel] != last_data[channel]) {
            last_change_times[channel] = t;
        }

        if (msg->data[channel] == last_data[channel] && param_timeout[channel] != 0)
            continue;

        if (msg->data[channel] > param_pwm_max[channel]) {
            set(channel, param_pwm_max[channel]);
        } else if (msg->data[channel] < param_pwm_min[channel]) {
            set(channel, param_pwm_min[channel]);
        } else {
            set(channel, msg->data[channel]);
        }
        last_set_times[channel] = t;
        last_data[channel] = msg->data[channel];
    }
}

}  // namespace pwm_pca9685
