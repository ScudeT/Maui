#include <chrono>
#include <memory>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "std_msgs/msg/float32.hpp"

using std::placeholders::_1;
using std::placeholders::_2;

class TriggerPublisherNode : public rclcpp::Node {
public:
  TriggerPublisherNode()
  : Node("trigger_publisher_node"), state_(false)
  {
    // Declare parameters
    this->declare_parameter<std::string>("topic_name", "trigger_topic");
    this->declare_parameter<float>("on_value", 1.0f);
    this->declare_parameter<float>("off_value", 0.0f);
    this->declare_parameter<float>("delay", 1.0f);

    // Get parameters
    this->get_parameter("topic_name", topic_name_);
    this->get_parameter("on_value", on_value_);
    this->get_parameter("off_value", off_value_);
    this->get_parameter("delay", delay_);

    // Create publisher
    publisher_ = this->create_publisher<std_msgs::msg::Float32>(topic_name_, 10);

    // Create single toggle service
    service_ = this->create_service<std_srvs::srv::Trigger>(
      "trigger", std::bind(&TriggerPublisherNode::toggle_callback, this, _1, _2));

    RCLCPP_INFO(this->get_logger(), "Ready: service 'trigger'; publishing on '%s'", topic_name_.c_str());
  }

private:
  void toggle_callback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    // Toggle state
    state_ = !state_;
    const char *state_str = state_ ? "ON" : "OFF";
    RCLCPP_INFO(this->get_logger(), "Received trigger: new state %s, waiting %.2f s...", state_str, delay_);

    // Delay before publishing
    std::this_thread::sleep_for(std::chrono::duration<float>(delay_));

    // Publish corresponding value
    auto msg = std_msgs::msg::Float32();
    msg.data = state_ ? on_value_ : off_value_;
    publisher_->publish(msg);
    RCLCPP_INFO(this->get_logger(), "Published %s value: %.2f", state_str, msg.data);

    response->success = true;
    response->message = std::string(state_str) + " value published";
  }

  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr publisher_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr service_;

  std::string topic_name_;
  float on_value_;
  float off_value_;
  float delay_;
  bool state_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TriggerPublisherNode>());
  rclcpp::shutdown();
  return 0;
}
