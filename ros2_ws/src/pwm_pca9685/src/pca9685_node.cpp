// pca9685_node.cpp
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>
#include "pwm_pca9685/pca9685.hpp"

class PcaNode : public rclcpp::Node {
public:
  PcaNode()
  : Node("pca9685_node"),
    pca_(get_logger(), "/dev/i2c-1", 0x40, 1000)
  {
    if (!pca_.begin()) {
      RCLCPP_FATAL(get_logger(), "Failed to initialize PCA9685");
      rclcpp::shutdown();
      return;
    }

    // Bind the member function as the callback
    subscription_ = create_subscription<std_msgs::msg::Int32MultiArray>(
      "command",
      rclcpp::QoS(10),
      std::bind(&PcaNode::commandCallback, this, std::placeholders::_1));
  }

private:
  // Separate callback function
  void commandCallback(const std_msgs::msg::Int32MultiArray::ConstSharedPtr msg)
  {
    if (msg->data.size() != 16) {
      RCLCPP_ERROR(get_logger(),
                   "Expected 16 values, got %zu", msg->data.size());
      return;
    }
    for (size_t i = 0; i < 16; ++i) {
      pca_.setDutyCycle(
        static_cast<uint8_t>(i),
        static_cast<uint16_t>(msg->data[i]));
    }
  }

  pwm_pca9685::PCA9685 pca_;
  rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr subscription_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PcaNode>());
  rclcpp::shutdown();
  return 0;
}
