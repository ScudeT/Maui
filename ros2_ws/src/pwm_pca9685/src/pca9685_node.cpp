#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>
#include <chrono>
#include <vector>
#include "pwm_pca9685/pca9685.hpp"

using namespace std::chrono_literals;

class PcaNode : public rclcpp::Node {
public:
  PcaNode()
  : Node("pca9685_node"),
    pca_(get_logger(), "/dev/i2c-1", 0x40, 300),
    last_msg_time_(this->now())
  {
    if (!pca_.start()) {
      RCLCPP_FATAL(get_logger(), "Failed to start PCA9685");
      rclcpp::shutdown();
      return;
    }

    subscription_ = create_subscription<std_msgs::msg::Int32MultiArray>(
      "command",
      rclcpp::QoS(10),
      std::bind(&PcaNode::commandCallback, this, std::placeholders::_1));

    timer_ = create_wall_timer(
      1s, std::bind(&PcaNode::checkTimeout, this));
  }

private:
  void commandCallback(const std_msgs::msg::Int32MultiArray::ConstSharedPtr msg) {
    if (msg->data.size() != 16) {
      RCLCPP_ERROR(get_logger(), "Expected 16 PWM values but got %zu", msg->data.size());
      return;
    }

    for (size_t i = 0; i < 16; ++i) {
      uint16_t pwm_val = static_cast<uint16_t>(msg->data[i]);
      pca_.set(static_cast<uint8_t>(i), pwm_val);
    }

    last_msg_time_ = this->now();  // Update last received message time
  }

  void checkTimeout() {
    auto now = this->now();
    if ((now - last_msg_time_).seconds() > 5.0) {
      // Timeout: send zeros to all channels
      RCLCPP_WARN_THROTTLE(get_logger(), *this->get_clock(), 1 /*ms*/,
                           "No command received in last 5 seconds, shutting down all channels");
      for (uint8_t ch = 0; ch < 16; ++ch) {
        pca_.set(ch, 0);
      }
      // Reset last_msg_time_ to now to avoid flooding this warning and reset commands
      last_msg_time_ = now;
    }
  }

  pwm_pca9685::PCA9685 pca_;
  rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr subscription_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Time last_msg_time_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PcaNode>());
  rclcpp::shutdown();
  return 0;
}
