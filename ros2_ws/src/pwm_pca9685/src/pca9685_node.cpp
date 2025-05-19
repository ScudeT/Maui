#include "rclcpp/rclcpp.hpp"
#include "pwm_pca9685/pca9685.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"

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
    sub_ = create_subscription<std_msgs::msg::Int32MultiArray>(
      "command", 10,
      [this](auto msg) {
        if (msg->data.size() != 16) {
          RCLCPP_ERROR(get_logger(), "Expected 16 values, got %zu", msg->data.size());
          return;
        }
        for (size_t ch = 0; ch < 16; ++ch) {
          pca_.setDutyCycle(ch, static_cast<uint16_t>(msg->data[ch]));
        }
      });
  }

private:
  pwm_pca9685::PCA9685 pca_;
  rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr sub_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PcaNode>());
  rclcpp::shutdown();
  return 0;
}
