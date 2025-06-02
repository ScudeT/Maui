#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>

// Include your controller libraries
#include "AttitudeController.hpp"
#include "PIDController.hpp"

class CtrlNode : public rclcpp::Node {
public:
    CtrlNode()
    : Node("ctrl_node"),
      // Set PID gains as you prefer here (example gains)
      pid_x_(1.0, 0.0, 0.0, 0.01, -1.0, 1.0, 0.5),
      pid_y_(1.0, 0.0, 0.0, 0.01, -1.0, 1.0, 0.5),
      pid_z_(1.0, 0.0, 0.0, 0.01, -1.0, 1.0, 0.5),
      att_ctrl_(tf2::Vector3(1.0, 1.0, 1.0), pid_x_, pid_y_, pid_z_)
    {
        // Parameter for frequency (Hz)
        publish_frequency_ = declare_parameter("publish_frequency", 10.0);

        // Publishers
        pub_alpha_d_ = create_publisher<std_msgs::msg::Float32>("alpha_d", 10);
        pub_theta_m_ = create_publisher<std_msgs::msg::Float32>("theta_m", 10);
        pub_A_d_     = create_publisher<std_msgs::msg::Float32>("A_d", 10);

        // Subscribers
        sub_measure_ = create_subscription<nav_msgs::msg::Odometry>(
            "measure", 10, std::bind(&CtrlNode::measure_callback, this, std::placeholders::_1));
        sub_setpoint_ = create_subscription<geometry_msgs::msg::Quaternion>(
            "setpoint", 10, std::bind(&CtrlNode::setpoint_callback, this, std::placeholders::_1));

        // Timer
        timer_ = create_wall_timer(
            std::chrono::duration<double>(1.0 / publish_frequency_),
            std::bind(&CtrlNode::timer_callback, this)
        );
    }

private:
    // Controller
    PIDController pid_x_, pid_y_, pid_z_;
    AttitudeController att_ctrl_;

    // Attributes (store last setpoint and measured quaternions and angular velocity)
    tf2::Quaternion q_set_;
    tf2::Quaternion q_mes_;
    tf2::Vector3    w_mes_;
    float alpha_d_{0.0f};
    float theta_m_{0.0f};
    float A_d_{0.0f};

    // Publishers & Subscribers
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_alpha_d_, pub_theta_m_, pub_A_d_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_measure_;
    rclcpp::Subscription<geometry_msgs::msg::Quaternion>::SharedPtr sub_setpoint_;
    rclcpp::TimerBase::SharedPtr timer_;
    double publish_frequency_;

    // Callbacks
    void setpoint_callback(const geometry_msgs::msg::Quaternion::SharedPtr msg) {
        q_set_ = tf2::Quaternion(msg->x, msg->y, msg->z, msg->w);
    }
    void measure_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        q_mes_ = tf2::Quaternion(msg->pose.pose.orientation.x,
                                  msg->pose.pose.orientation.y,
                                  msg->pose.pose.orientation.z,
                                  msg->pose.pose.orientation.w);
        w_mes_ = tf2::Vector3(msg->twist.twist.angular.x,
                               msg->twist.twist.angular.y,
                               msg->twist.twist.angular.z);
    }

    void timer_callback() {
        get_ctrl(); // update alpha_d_, theta_m_, A_d_

        std_msgs::msg::Float32 msg_alpha, msg_theta, msg_A;
        msg_alpha.data = alpha_d_;
        msg_theta.data = theta_m_;
        msg_A.data     = A_d_;

        pub_alpha_d_->publish(msg_alpha);
        pub_theta_m_->publish(msg_theta);
        pub_A_d_->publish(msg_A);
    }

    void get_ctrl() {

        // Compute control using the attitude controller
        tf2::Vector3 u = att_ctrl_.get_u(q_set_, q_mes_, w_mes_);

        // Assign to published variables (example: map directly)
        alpha_d_ = u.x();
        theta_m_ = u.y();
        A_d_     = u.z();
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CtrlNode>());
    rclcpp::shutdown();
    return 0;
}
