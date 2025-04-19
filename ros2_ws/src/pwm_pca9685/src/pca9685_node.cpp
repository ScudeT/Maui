#include "rclcpp/rclcpp.hpp"
#include "pwm_pca9685/pca9685_activity.h"

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  
  // Create node and activity instance in a local scope.
  auto node = rclcpp::Node::make_shared("pca9685_node");
  auto activity = std::make_shared<pwm_pca9685::PCA9685Activity>(node);

  if (!activity->start()) {
    RCLCPP_ERROR(node->get_logger(), "Failed to start activity");
    rclcpp::shutdown();
    return -4;
  }

  // Create a timer that calls spinOnce() at 100 Hz (every 10ms)
  // Declare the 'freq' parameter with a default value (e.g., 100 Hz)
  int freq = node->declare_parameter("freq", 100);
  // Compute the timer period in milliseconds: period = 1000/freq
  auto timer_period = std::chrono::milliseconds(1000 / freq);

  auto timer = node->create_wall_timer(
    timer_period,
    [&]() {
      activity->spinOnce();
    }
  );

  // Use rclcpp::spin to process callbacks (including the timer)
  rclcpp::spin(node);

  // When rclcpp::spin returns (e.g. due to SIGINT), cleanly stop the activity
  activity->stop();
  
  rclcpp::shutdown();
  return 0;
}
