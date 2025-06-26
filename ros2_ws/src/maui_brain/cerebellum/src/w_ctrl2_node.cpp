#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>
#include "std_srvs/srv/trigger.hpp"

// Include your controller libraries
#include "PIDController.hpp"

#define TODEGREE  180.0 / M_PI

class WControllerNode : public rclcpp::Node {
public:
    WControllerNode()
    : Node("ctrl_node")
    {
        // Parameter for frequency (Hz)
        int update_rate = declare_parameter("freq", 10);
        int period_ms = static_cast<int>(1000.0 / update_rate);
        double dt_ = period_ms / 1000.0;


        // PID initializations
        // settings are: // {Kp, Ki, Kd, dt, output_min, output_max, deriv_filter_coef}
        RCLCPP_INFO(this->get_logger(), "Controller node started with update rate: %d Hz", update_rate);
        std::vector<double> roll_pid_settings = declare_parameter<std::vector<double>>("wx_pid_settings", std::vector<double>{10.0, 0.0, 0.0, -30.0, 30.0, 0.5});
        pid_x_ = std::make_shared<PIDController>(roll_pid_settings[0], roll_pid_settings[1], roll_pid_settings[2],
                                                  dt_, roll_pid_settings[3], roll_pid_settings[4], roll_pid_settings[5]);
 
        std::vector<double> pitch_pid_settings = declare_parameter<std::vector<double>>("wy_pid_settings", std::vector<double>{10.0, 0.0, 0.0, -30.0, 30.0, 0.5});
        pid_y_ = std::make_shared<PIDController>(pitch_pid_settings[0], pitch_pid_settings[1], pitch_pid_settings[2],
                                                  dt_, pitch_pid_settings[3], pitch_pid_settings[4], pitch_pid_settings[5]);

        std::vector<double> yaw_pid_settings = declare_parameter<std::vector<double>>("wz_pid_settings", std::vector<double>{10.0, 0.0, 0.0, -30.0, 30.0, 0.5});
        pid_z_ = std::make_shared<PIDController>(yaw_pid_settings[0], yaw_pid_settings[1], yaw_pid_settings[2],
                                                  dt_, yaw_pid_settings[3], yaw_pid_settings[4], yaw_pid_settings[5]);
        //pid_x_->setVerbose(true);
        //pid_y_->setVerbose(true);
        pid_z_->setVerbose(true);

        // Publishers
        pub_w_act_ = create_publisher<geometry_msgs::msg::Vector3>("w_actuation", 10);

        // Subscribers
        measure_sub_ = create_subscription<nav_msgs::msg::Odometry>(
            "measure", 10, std::bind(&WControllerNode::measure_callback, this, std::placeholders::_1));
        setpoint_sub_ = create_subscription<geometry_msgs::msg::Vector3>(
            "setpoint", 10, std::bind(&WControllerNode::setpoint_callback, this, std::placeholders::_1));
        
        // Timer
        timer_ = create_wall_timer(
            std::chrono::milliseconds(period_ms),
            std::bind(&WControllerNode::timer_callback, this)
        );

        int update_freq = this->declare_parameter("update_freq", 1);
        int update_period_ms = static_cast<int>(1000.0 / update_freq);
        update_ = this->create_wall_timer(
        std::chrono::milliseconds(update_period_ms),
        std::bind(&WControllerNode::updateParams, this));

        reset_service_ = this->create_service<std_srvs::srv::Trigger>(
                            "reset_controllers",
                            std::bind(&WControllerNode::resetCallback, this, std::placeholders::_1, std::placeholders::_2));
    }

private:
    // Controller
    std::shared_ptr<PIDController> pid_x_;
    std::shared_ptr<PIDController> pid_y_;
    std::shared_ptr<PIDController> pid_z_;

    // Attributes (store last setpoint and measured quaternions and angular velocity)
    geometry_msgs::msg::Vector3  w_mes_;
    geometry_msgs::msg::Vector3  w_set_;

    // Publishers & Subscribers
    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr pub_w_act_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr measure_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr setpoint_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::TimerBase::SharedPtr update_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_service_;
    double dt_;

    // Callbacks
    void setpoint_callback(const geometry_msgs::msg::Vector3::SharedPtr msg) {
        w_set_ = *msg;
    }

    void measure_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        w_mes_ = msg->twist.twist.angular;
    }

    void timer_callback() {
        // Publish the current setpoint and measured values
        geometry_msgs::msg::Vector3 msg;
        // Compute the PID control outputs
        // Note: The PIDController expects the input in degrees, so we convert from radians to degrees
        double set_x = w_set_.x * TODEGREE;
        double set_y = w_set_.y * TODEGREE;
        double set_z = w_set_.z * TODEGREE;
        double mes_x = w_mes_.x * TODEGREE;
        double mes_y = w_mes_.y * TODEGREE;
        double mes_z = w_mes_.z * TODEGREE;

        double wx_act = pid_x_->compute(set_x, mes_x);
        double wy_act = pid_y_->compute(set_y, mes_y);
        double wz_act = pid_z_->compute(set_z, mes_z);

        msg.x = wx_act;
        msg.y = wy_act;
        msg.z = wz_act;

        pub_w_act_->publish(msg);
        RCLCPP_INFO(this->get_logger(), "Published x: %0.2f, y: %0.2f, z: %0.2f", 
                                    msg.x, msg.y, msg.z);
    }

    void resetCallback(const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
                        std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        pid_x_->reset();
        pid_y_->reset();
        pid_z_->reset();
        response->success = true;
        response->message = "w PID controller state has been reset.";
        RCLCPP_INFO(this->get_logger(), "w PID controller state has been reset.");
    }

    void updateParams() {
        auto pid_settings_x = this->get_parameter("wx_pid_settings").as_double_array();
        if (pid_settings_x.size() != 6) {
            RCLCPP_FATAL(this->get_logger(), "wx_pid_settings must contain exactly 6 values.");
        }else{
            pid_x_->updateCoefficients(pid_settings_x[0], pid_settings_x[1], pid_settings_x[2],
                                    pid_settings_x[3], pid_settings_x[4], pid_settings_x[5]);
        }

        auto pid_settings_y = this->get_parameter("wy_pid_settings").as_double_array();
        if (pid_settings_y.size() != 6) {
            RCLCPP_FATAL(this->get_logger(), "wy_pid_settings must contain exactly 6 values.");
        }else{
            pid_y_->updateCoefficients(pid_settings_y[0], pid_settings_y[1], pid_settings_y[2],
                                     pid_settings_y[3], pid_settings_y[4], pid_settings_y[5]);
        }

        auto pid_settings_z = this->get_parameter("wz_pid_settings").as_double_array();
        if (pid_settings_z.size() != 6) {
            RCLCPP_FATAL(this->get_logger(), "wz_pid_settings must contain exactly 6 values.");
            return;
        }else{
            pid_z_->updateCoefficients(pid_settings_z[0], pid_settings_z[1], pid_settings_z[2],
                                    pid_settings_z[3], pid_settings_z[4], pid_settings_z[5]);
        }
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<WControllerNode>());
    rclcpp::shutdown();
    return 0;
}
