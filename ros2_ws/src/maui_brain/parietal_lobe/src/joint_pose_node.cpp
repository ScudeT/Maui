#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include <vector>
#include <string>
#include <cmath>

class JointStatePublisher : public rclcpp::Node
{
public:
    JointStatePublisher()
    : Node("joint_state_publisher")
    {
        // Declare and get pwm_range parameter (simplified)
        this->declare_parameter<std::vector<int64_t>>("pwm_range", {21500, 51500});
        pwm_range_ = this->get_parameter("pwm_range").as_integer_array();

        pwm_d_ = pwm_range_[1] - pwm_range_[0];
        pwm_min_ = pwm_range_[0];

         // Declare and get servo_range parameter (simplified)
        this->declare_parameter<std::vector<double>>("servo_range", {-60.0, 60.0});
        servo_range_ = this->get_parameter("servo_range").as_double_array();
        servo_d_ = servo_range_[1] - servo_range_[0];
        servo_min_ = servo_range_[0];

        // Publisher
        publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);

        // Subscription
        subscription_ = this->create_subscription<std_msgs::msg::Int32MultiArray>(
            "command", 10,
            std::bind(&JointStatePublisher::listener_callback, this, std::placeholders::_1)
        );

        // Initialize joint names and positions
        joint_state_msg_.name = {"finl_joint", "wingl_joint", "wingr_joint", "finr_joint"};
        joint_state_msg_.position = {0.0, 0.0, 0.0, 0.0};
        // Update message and publish
        rclcpp::sleep_for(std::chrono::milliseconds(100)); // Allow time for parameters to be set
        joint_state_msg_.header.stamp = this->now();
        publisher_->publish(joint_state_msg_);
    }

private:
    void listener_callback(const std_msgs::msg::Int32MultiArray::SharedPtr msg)
    {
        std::vector<int32_t> pwm_data(msg->data.begin(), msg->data.end());
        // Get slice [12:] (from index 12 onwards)
        if (pwm_data.size() < 16) return; // Defensive: must have at least 16 elements
        pwm_data = std::vector<int32_t>(pwm_data.begin() + 12, pwm_data.end());

        std::vector<double> joint_position;
        for (const auto& pwm : pwm_data)
            joint_position.push_back(pwmToAngle(pwm));

        // Update message and publish
        joint_state_msg_.position = joint_position;
        joint_state_msg_.header.stamp = this->now();
        publisher_->publish(joint_state_msg_);
    }

    double pwmToAngle(int pwm)
    {
        double angle = static_cast<double>(pwm - pwm_min_) / pwm_d_ * servo_d_ + servo_min_;
        angle = angle * M_PI / 180.0;
        return angle;
    }

    // Parameters
    std::vector<int64_t> pwm_range_;
    int pwm_min_;
    int pwm_d_;
    std::vector<double> servo_range_;
    double servo_min_;
    double servo_d_;

    // ROS
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr publisher_;
    rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr subscription_;

    // Message
    sensor_msgs::msg::JointState joint_state_msg_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<JointStatePublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
