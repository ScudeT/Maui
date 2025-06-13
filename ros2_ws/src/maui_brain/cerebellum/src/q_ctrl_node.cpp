#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>

// Include your controller libraries
#include "PIDController.hpp"

class QControllerNode : public rclcpp::Node {
public:
    QControllerNode()
    : Node("ctrl_node")
    {
        // Parameter for frequency (Hz)
        int update_rate = declare_parameter("freq", 10);
        int period_ms = static_cast<int>(1000.0 / update_rate);

        // Attitude controller gains
        Kp_ = declare_parameter("attitude_gains", 1.0);

        // Publishers
        pub_w_set_ = create_publisher<geometry_msgs::msg::Vector3>("w_set", 10);

        // Subscribers
        measure_sub_ = create_subscription<nav_msgs::msg::Odometry>(
            "measure", 10, std::bind(&QControllerNode::measure_callback, this, std::placeholders::_1));
        setpoint_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
            "setpoint", 10, std::bind(&QControllerNode::setpoint_callback, this, std::placeholders::_1));

        // Timer
        timer_ = create_wall_timer(
            std::chrono::milliseconds(period_ms),
            std::bind(&QControllerNode::timer_callback, this)
        );
    }

private:
    double Kp_; // Proportional gains for attitude control

    // Attributes (store last setpoint and measured quaternions and angular velocity)
    tf2::Quaternion  q_mes_;
    tf2::Quaternion  q_set_;

    // Publishers & Subscribers
    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr pub_w_set_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr measure_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr setpoint_sub_;
    rclcpp::TimerBase::SharedPtr timer_;

    // Callbacks
    void setpoint_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        const auto& q = msg->pose.orientation;
        q_set_ = tf2::Quaternion(q.x, q.y, q.z, q.w);
    }


    void measure_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        q_mes_ = tf2::Quaternion(msg->pose.pose.orientation.x,
                                msg->pose.pose.orientation.y,
                                msg->pose.pose.orientation.z,
                                msg->pose.pose.orientation.w);
    }

    void timer_callback() {
        // Compute quaternion error
        tf2::Quaternion q_err =  q_set_*q_mes_.inverse();

        // Get shortest rotation
        if (q_err.getW() < 0) {
            q_err = tf2::Quaternion(-q_err.getX(), -q_err.getY(), -q_err.getZ(), -q_err.getW());
        }

        // Compute angular velocity setpoint from quaternion error
        // tf2::Vector3 w_sp = 2.0 * Kp_ * q_err.getAxis() * q_err.getAngle();
        tf2::Quaternion q_u =  tf2::Quaternion(0, 0, 0, 1);
        tf2::Quaternion w_sp_g =  q_u.inverse()*q_err;
        tf2::Quaternion w_sp =  q_mes_.inverse()*w_sp_g*q_mes_;


        tf2::Vector3 w_sp_vec(-2.0*Kp_*w_sp.getX(), -2.0*Kp_*w_sp.getY(), -2.0*Kp_*w_sp.getZ());

        // Publish the computed angular velocity setpoint
        geometry_msgs::msg::Vector3 w_set_msg;
        w_set_msg.x = w_sp_vec.getX();
        w_set_msg.y = w_sp_vec.getY();
        w_set_msg.z = w_sp_vec.getZ();
        pub_w_set_->publish(w_set_msg);
        RCLCPP_INFO(this->get_logger(), "w_sp: [%f, %f, %f, %f]", w_sp.getX(), w_sp.getY(), w_sp.getZ(), w_sp.getW());
        RCLCPP_INFO(this->get_logger(), "q_err: [%f, %f, %f, %f]", q_err.getX(), q_err.getY(), q_err.getZ(), q_err.getW());
        RCLCPP_INFO(this->get_logger(), "q_mes: [%f, %f, %f, %f]", q_mes_.getX(), q_mes_.getY(), q_mes_.getZ(), q_mes_.getW());
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<QControllerNode>());
    rclcpp::shutdown();
    return 0;
}
