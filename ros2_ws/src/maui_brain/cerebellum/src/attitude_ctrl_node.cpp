#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>
#include "std_srvs/srv/trigger.hpp"

// Include your controller libraries
#include "AttitudeController.hpp"
#include "PIDController.hpp"

class AttCtrlNode : public rclcpp::Node {
public:
    AttCtrlNode()
    : Node("ctrl_node")
    {
        // Parameter for frequency (Hz)
        int update_rate = declare_parameter("freq", 10);
        int period_ms = static_cast<int>(1000.0 / update_rate);
        double period_s = period_ms / 1000.0;

        // Attitude controller gains
        std::vector<double> att_ctrl_settings = declare_parameter<std::vector<double>>("attitude_gains", std::vector<double>{1.0, 1.0, 1.0});
        tf2::Vector3 att_gain(att_ctrl_settings[0], att_ctrl_settings[1], att_ctrl_settings[2]);


        // PID initializations
        // settings are: // {Kp, Ki, Kd, dt, output_min, output_max, deriv_filter_coef}
        RCLCPP_INFO(this->get_logger(), "Controller node started with update rate: %d Hz", update_rate);
        std::vector<double> wx_pid_settings = declare_parameter<std::vector<double>>("wx_pid_settings", std::vector<double>{10.0, 0.0, 0.0, -30.0, 30.0, 0.5});
        PIDController pid_x(wx_pid_settings[0], wx_pid_settings[1], wx_pid_settings[2],
                            period_s, wx_pid_settings[3], wx_pid_settings[4], wx_pid_settings[5]);
 
        std::vector<double> wy_pid_settings = declare_parameter<std::vector<double>>("wy_pid_settings", std::vector<double>{10.0, 0.0, 0.0, -30.0, 30.0, 0.5});
        PIDController pid_y(wy_pid_settings[0], wy_pid_settings[1], wy_pid_settings[2],
                            period_s, wy_pid_settings[3], wy_pid_settings[4], wy_pid_settings[5]);
        
        std::vector<double> wz_pid_settings = declare_parameter<std::vector<double>>("wz_pid_settings", std::vector<double>{10.0, 0.0, 0.0, -30.0, 30.0, 0.5});
        PIDController pid_z(wz_pid_settings[0], wz_pid_settings[1], wz_pid_settings[2],
                            period_s, wz_pid_settings[3], wz_pid_settings[4], wz_pid_settings[5]);
        
        //pid_x.setVerbose(true);
        //pid_y.setVerbose(true);
        pid_z.setVerbose(true);

        // Attitude controller initialization
        att_ctrl_ = std::make_shared<AttitudeController>(att_gain, pid_x, pid_y, pid_z);

        // Publishers
        pub_wx_act_ = create_publisher<std_msgs::msg::Float32>("wx_act", 10);
        pub_wy_act = create_publisher<std_msgs::msg::Float32>("wy_act", 10);
        pub_wz_act_     = create_publisher<std_msgs::msg::Float32>("wz_act", 10);

        // Subscribers
        sub_measure_ = create_subscription<nav_msgs::msg::Odometry>(
            "measure", 10, std::bind(&AttCtrlNode::measure_callback, this, std::placeholders::_1));
        sub_setpoint_ = create_subscription<geometry_msgs::msg::Quaternion>(
            "setpoint", 10, std::bind(&AttCtrlNode::setpoint_callback, this, std::placeholders::_1));
        
        // Timer
        timer_ = create_wall_timer(
            std::chrono::milliseconds(period_ms),
            std::bind(&AttCtrlNode::timer_callback, this)
        );

        reset_service_ = this->create_service<std_srvs::srv::Trigger>(
                            "reset_controllers",
                            std::bind(&AttCtrlNode::resetCallback, this, std::placeholders::_1, std::placeholders::_2));
    }

private:
    // Controller
    std::shared_ptr<AttitudeController> att_ctrl_;

    // Attributes (store last setpoint and measured quaternions and angular velocity)
    tf2::Quaternion q_set_;
    tf2::Quaternion q_mes_;
    tf2::Vector3    w_mes_;
    double wx_act_{0.0};
    double wy_act_{0.0};
    double wz_act_{0.0};

    // Publishers & Subscribers
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_wx_act_, pub_wy_act, pub_wz_act_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_measure_;
    rclcpp::Subscription<geometry_msgs::msg::Quaternion>::SharedPtr sub_setpoint_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_service_;

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
        get_ctrl(); // update wx_act_, wy_act_, wz_act_

        std_msgs::msg::Float32 msg_wx_act, msg_wy_act, msg_wz_act;
        msg_wx_act.data = wx_act_;
        msg_wy_act.data = wy_act_;
        msg_wz_act.data     = wz_act_;

        pub_wx_act_->publish(msg_wx_act);
        pub_wy_act->publish(msg_wy_act);
        pub_wz_act_->publish(msg_wz_act);
    }

    void get_ctrl() {

        // Compute control using the attitude controller
        tf2::Vector3 u = att_ctrl_->get_u(q_set_, q_mes_, w_mes_);

        // Assign to published variables (example: map directly)
        wx_act_ = u.x();
        wy_act_ = u.y();
        wz_act_ = u.z();
    }

    void resetCallback(const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
                        std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        att_ctrl_->reset();
        response->success = true;
        response->message = "w PID controller state has been reset.";
        RCLCPP_INFO(this->get_logger(), "w PID controller state has been reset.");
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<AttCtrlNode>());
    rclcpp::shutdown();
    return 0;
}
