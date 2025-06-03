#include <chrono>
#include <memory>
#include <string>
#include <functional>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "PIDController.hpp"

using namespace std::chrono_literals;

class PIDControllerNode : public rclcpp::Node {
public:
  PIDControllerNode()
  : Node("pid_controller_node")
  {
    std::vector<double> default_pid = {10.0, 0.0, 0.0, -30.0, 30.0, 0.5};
    this->declare_parameter<std::vector<double>>("pid_settings", default_pid);
    this->declare_parameter<std::string>("feedback_field", "x");
    this->declare_parameter<int>("freq", 10); // Hz

    auto pid_settings = this->get_parameter("pid_settings").as_double_array();
    if (pid_settings.size() != 6) {
      RCLCPP_FATAL(this->get_logger(), "pid_settings must contain exactly 6 values.");
      rclcpp::shutdown();
    }

    double kp = pid_settings[0];
    double ki = pid_settings[1];
    double kd = pid_settings[2];
    double min = pid_settings[3];
    double output_max = pid_settings[4];
    double deriv_filter_coef = pid_settings[5];
    int frequency = this->get_parameter("freq").as_int();
    double dt = 1.0 / frequency;

    ref_sub_ = this->create_subscription<std_msgs::msg::Float32>(
      "reference", 10,
      [this](std_msgs::msg::Float32::SharedPtr msg) {
        setpoint_ = msg->data;
      });

    feedback_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "measurement", 10,
      [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
        std::string field = this->get_parameter("feedback_field").as_string();
        feedback_ = extractField(*msg, field);
      });

    control_pub_ = this->create_publisher<std_msgs::msg::Float32>("control_value", 10);

    pid_ = std::make_shared<PIDController>(kp, ki, kd, dt, min, output_max, deriv_filter_coef);

    timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::duration<double>(dt)),
      std::bind(&PIDControllerNode::timerCallback, this));

    reset_service_ = this->create_service<std_srvs::srv::Trigger>(
      "reset_controller",
      std::bind(&PIDControllerNode::resetCallback, this, std::placeholders::_1, std::placeholders::_2));
  }

private:
  double extractField(const nav_msgs::msg::Odometry& msg, const std::string& field) {
    if (field == "x") return msg.pose.pose.position.x;
    if (field == "y") return msg.pose.pose.position.y;
    if (field == "z") return msg.pose.pose.position.z;

    if (field == "vx") return msg.twist.twist.linear.x;
    if (field == "vy") return msg.twist.twist.linear.y;
    if (field == "vz") return msg.twist.twist.linear.z;

    if (field == "wx") return msg.twist.twist.angular.x;
    if (field == "wy") return msg.twist.twist.angular.y;
    if (field == "wz") return msg.twist.twist.angular.z;

    tf2::Quaternion q(
      msg.pose.pose.orientation.x,
      msg.pose.pose.orientation.y,
      msg.pose.pose.orientation.z,
      msg.pose.pose.orientation.w);
    tf2::Matrix3x3 m(q);

    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    if (field == "roll") return roll;
    if (field == "pitch") return pitch;
    if (field == "yaw") return yaw;

    RCLCPP_WARN(this->get_logger(), "Unknown feedback_field: %s. Returning 0.", field.c_str());
    return 0.0;
  }

  void timerCallback() {
    double control_value = pid_->compute(setpoint_, feedback_);
    auto msg = std_msgs::msg::Float32();
    msg.data = control_value;
    control_pub_->publish(msg);
  }

  void resetCallback(const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
                     std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
    pid_->reset();
    response->success = true;
    response->message = "PID controller state has been reset.";
    RCLCPP_INFO(this->get_logger(), "PID controller state has been reset.");
  }

  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr ref_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr feedback_sub_;
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
