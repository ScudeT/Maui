#include <chrono>
#include <memory>
#include <string>
#include <functional>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_srvs/srv/trigger.hpp"  // Changed from Empty to Trigger
#include "PIDController.hpp"  // Note the include reflects the directory structure

using namespace std::chrono_literals;

class PIDControllerNode : public rclcpp::Node {
public:
  PIDControllerNode()
  : Node("pid_controller_node")
  {
    // Declare dynamic parameters for the PID gains and limits.
    this->declare_parameter<double>("kp", 1.0);
    this->declare_parameter<double>("ki", 0.5);
    this->declare_parameter<double>("kd", 0.1);
    this->declare_parameter<double>("min", -10.0);
    this->declare_parameter<double>("output_max", 10.0);
    this->declare_parameter<double>("deriv_filter_coef", 0.8);

    // Declare a parameter for the control frequency (static at runtime).
    this->declare_parameter<double>("freq", 100.0); // Hz

    double frequency = this->get_parameter("freq").as_double();
    double dt = 1.0 / frequency;

    // Subscribers.
    ref_sub_ = this->create_subscription<std_msgs::msg::Float32>(
      "reference", 10,
      [this](std_msgs::msg::Float32::SharedPtr msg) {
        setpoint_ = msg->data;
      });
      
    feedback_sub_ = this->create_subscription<std_msgs::msg::Float32>(
      "feedback", 10,
      [this](std_msgs::msg::Float32::SharedPtr msg) {
        feedback_ = msg->data;
        updateControllerParameters();
      });
      
    // Publisher.
    control_pub_ = this->create_publisher<std_msgs::msg::Float32>("control_value", 10);

    // Initialize PID controller with current parameters.
    double kp = this->get_parameter("kp").as_double();
    double ki = this->get_parameter("ki").as_double();
    double kd = this->get_parameter("kd").as_double();
    double min = this->get_parameter("min").as_double();
    double output_max = this->get_parameter("output_max").as_double();
    double deriv_filter_coef = this->get_parameter("deriv_filter_coef").as_double();
    pid_ = std::make_shared<PIDController>(kp, ki, kd, dt, min, output_max, deriv_filter_coef);

    // Timer for periodic control computation.
    timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::duration<double>(dt)),
      std::bind(&PIDControllerNode::timerCallback, this));

    // Service to reset the controller using Trigger.
    reset_service_ = this->create_service<std_srvs::srv::Trigger>(
      "reset_controller",
      std::bind(&PIDControllerNode::resetCallback, this, std::placeholders::_1, std::placeholders::_2));
  }

private:
  void updateControllerParameters() {
    double kp = this->get_parameter("kp").as_double();
    double ki = this->get_parameter("ki").as_double();
    double kd = this->get_parameter("kd").as_double();
    double min = this->get_parameter("min").as_double();
    double output_max = this->get_parameter("output_max").as_double();
    double deriv_filter_coef = this->get_parameter("deriv_filter_coef").as_double();
    double frequency = this->get_parameter("freq").as_double();
    double dt = 1.0 / frequency;
    pid_->updateCoefficients(kp, ki, kd, dt, min, output_max, deriv_filter_coef);
  }
  
  void timerCallback() {
    double control_value = pid_->compute(setpoint_, feedback_);
    auto msg = std_msgs::msg::Float32();
    msg.data = control_value;
    control_pub_->publish(msg);
  }
  
  // Service callback to reset the PID controller's state.
  void resetCallback(const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
                     std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
    pid_->reset();
    response->success = true;
    response->message = "PID controller state has been reset.";
    RCLCPP_INFO(this->get_logger(), "PID controller state has been reset.");
  }
  
  // Node members.
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr ref_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr feedback_sub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr control_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_service_;
  std::shared_ptr<PIDController> pid_;
  
  double setpoint_{0.0};
  double feedback_{0.0};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PIDControllerNode>());
  rclcpp::shutdown();
  return 0;
}
