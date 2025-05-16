#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_srvs/srv/trigger.hpp"
#include <cmath>
#include <vector>
#include <algorithm>

#define LEFT_WING   13
#define RIGHT_WING  14

#define LEFT_FLAP   12
#define RIGHT_FLAP  15

using std::placeholders::_1;
using std::placeholders::_2;

class CommandPublisher : public rclcpp::Node
{
public:
  CommandPublisher()
  : Node("command_publisher")
  {
    // Declare parameters with default values
    this->declare_parameter<int>("freq", 30);
    this->declare_parameter<std::vector<int>>("angle_range", std::vector<int>{-60, 60});
    this->declare_parameter<std::vector<int>>("pwm_range", std::vector<int>{15000, 40000});
    this->declare_parameter<std::vector<double>>("start_wing", std::vector<double>{1.0, 60.0, 0.0, 0.0});
    // start_flap now holds the starting values for flap_same and flap_dif
    this->declare_parameter<std::vector<double>>("start_flap", std::vector<double>{0.0, 0.0});
    // New parameter for clamping flap angles
    this->declare_parameter<std::vector<int>>("clamp_flap", std::vector<int>{-40, 40});

    // Retrieve parameters
    timer_frequency_ = this->get_parameter("freq").as_int();

    auto angle_range = this->get_parameter("angle_range").as_integer_array();
    if (angle_range.size() >= 2) {
      angle_min_ = angle_range[0];
      angle_max_ = angle_range[1];
    } else {
      angle_min_ = -60;
      angle_max_ = 60;
    }

    auto pwm_range = this->get_parameter("pwm_range").as_integer_array();
    if (pwm_range.size() >= 2) {
      pwm_min_ = pwm_range[0];
      pwm_max_ = pwm_range[1];
    } else {
      pwm_min_ = 15000;
      pwm_max_ = 40000;
    }

    auto start_wing = this->get_parameter("start_wing").as_double_array();
    if (start_wing.size() >= 3) {
      omega_m_ = start_wing[0];
      A_m_  = start_wing[1];
      A_d_ = start_wing[2];
      alpha_d_ = start_wing[3];
    } else {
      omega_m_ = 1.0;
      A_m_  = 0.0;
      A_d_ = 0.0;
      alpha_d_ = 0.0;
    }

    auto start_flap = this->get_parameter("start_flap").as_double_array();
    if (start_flap.size() >= 2) {
      theta_m_ = start_flap[0];
      theta_d_  = start_flap[1];
    } else {
      theta_m_ = 0.0;
      theta_d_  = 0.0;
    }

    auto clamp_flap = this->get_parameter("clamp_flap").as_integer_array();
    if (clamp_flap.size() >= 2) {
      clamp_flap_min_ = clamp_flap[0];
      clamp_flap_max_ = clamp_flap[1];
    } else {
      clamp_flap_min_ = -40;
      clamp_flap_max_ = 40;
    }

    // Initialize publishing flag (true = publishing enabled)
    publishing_enabled_ = false;

    // Subscribers for wing parameters
    omega_m_sub_ = this->create_subscription<std_msgs::msg::Float32>(
      "/freq", 10, std::bind(&CommandPublisher::omegaMCallback, this, _1));
    A_m_sub_ = this->create_subscription<std_msgs::msg::Float32>(
      "/A_m", 10, std::bind(&CommandPublisher::AmCallback, this, _1));
    A_d_sub_ = this->create_subscription<std_msgs::msg::Float32>(
      "/A_d", 10, std::bind(&CommandPublisher::AdCallback, this, _1));
    alpha_d_sub_ = this->create_subscription<std_msgs::msg::Float32>(
      "/alpha_d", 10, std::bind(&CommandPublisher::alphaDCallback, this, _1));

    // Subscribers for flap topics (using the new two-topic approach)
    theta_m_sub_ = this->create_subscription<std_msgs::msg::Float32>(
      "/theta_m", 10, std::bind(&CommandPublisher::thetaMCallback, this, _1));
    theta_d_sub_ = this->create_subscription<std_msgs::msg::Float32>(
      "/theta_d", 10, std::bind(&CommandPublisher::thetaDCallback, this, _1));

    // Publisher for /command topic
    command_pub_ = this->create_publisher<std_msgs::msg::Int32MultiArray>("/command", 10);

    // Service to start/stop publishing command messages
    start_stop_srv_ = this->create_service<std_srvs::srv::Trigger>(
      "start_and_stop", std::bind(&CommandPublisher::handleStartStop, this, _1, _2));

    // Timer callback using the timer frequency parameter (Hz)
    timer_ = this->create_wall_timer(
      std::chrono::duration<double>(1.0 / timer_frequency_),
      std::bind(&CommandPublisher::timerCallback, this));

    start_time_ = this->now();
  }

private:
  // Wing parameter callbacks
  void omegaMCallback(const std_msgs::msg::Float32::SharedPtr msg)
  {
    omega_m_ = msg->data;
  }

  void AmCallback(const std_msgs::msg::Float32::SharedPtr msg)
  {
    A_m_ = msg->data;
  }

  void AdCallback(const std_msgs::msg::Float32::SharedPtr msg)
  {
    A_d_ = msg->data;
  }

  void wingDiffCallback(const std_msgs::msg::Float32::SharedPtr msg)
  {
    A_d_ = msg->data;
  }

  void alphaDCallback(const std_msgs::msg::Float32::SharedPtr msg)
  {
    alpha_d_ = msg->data;
  }

  // Flap parameter callbacks (new topics)
  void thetaMCallback(const std_msgs::msg::Float32::SharedPtr msg)
  {
    theta_m_ = msg->data;
  }

  void thetaDCallback(const std_msgs::msg::Float32::SharedPtr msg)
  {
    theta_d_ = msg->data;
  }

  // Service callback: toggles publishing on/off.
  void handleStartStop(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                       std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    (void)request;  // request is not used
    publishing_enabled_ = !publishing_enabled_;
    if (publishing_enabled_) {
      response->message = "Publishing started.";
    } else {
      response->message = "Publishing stopped.";
    }
    response->success = true;
    RCLCPP_INFO(this->get_logger(), "%s", response->message.c_str());
  }

  // Mapping function for wing angles:
  // If the wing angle is outside the angle_range, returns 0.
  int32_t mapAngleToPWM(double angle)
  {
    if (angle < angle_min_ || angle > angle_max_) {
      return 0;
    }
    double pwm = ((angle - angle_min_) / static_cast<double>(angle_max_ - angle_min_)) * (pwm_max_ - pwm_min_) + pwm_min_;
    return static_cast<int32_t>(pwm);
  }

  // Mapping function for flap angles:
  // Clamps the angle to the clamp_flap range and linearly maps it to the PWM range.
  int32_t mapFlapAngleToPWM(double angle)
  {
    angle = std::max(static_cast<double>(clamp_flap_min_), std::min(angle, static_cast<double>(clamp_flap_max_)));
    double pwm = ((angle - clamp_flap_min_) / static_cast<double>(clamp_flap_max_ - clamp_flap_min_)) * (pwm_max_ - pwm_min_) + pwm_min_;
    return static_cast<int32_t>(pwm);
  }

  void timerCallback()
  {
    // Check if publishing is enabled; if not, skip publishing.
    if (!publishing_enabled_) {
      RCLCPP_DEBUG(this->get_logger(), "Publishing is currently disabled.");
      return;
    }

    // Compute elapsed time in seconds
    auto current_time = this->now();
    double t = (current_time - start_time_).seconds();

    // Compute the wing motion using a sine function.
    double half_amp = A_m_ / 2.0;
    double sine_val = sin(2 * M_PI * omega_m_ * t);

    // Differential adjustment for the wings:
    double left_wing_angle  = (half_amp - A_d_ / 2.0) * sine_val + std::max(0.0, alpha_d_);
    double right_wing_angle = - (half_amp + A_d_ / 2.0) * sine_val - std::min(0.0, alpha_d_);

    // Compute flap angles:
    // flap_dif is added to both, while flap_same is added to the left and subtracted from the right.
    // double left_flap_angle  = theta_d_ - theta_m_;
    // double right_flap_angle = theta_d_ + theta_m_;

    double left_flap_angle  = std::max(0.0, theta_d_) - theta_m_;
    double right_flap_angle = std::min(0.0, theta_d_) + theta_m_;

    // Map computed angles to PWM values
    int32_t left_wing_pwm  = mapAngleToPWM(left_wing_angle);
    int32_t right_wing_pwm = mapAngleToPWM(right_wing_angle);
    int32_t left_flap_pwm  = mapFlapAngleToPWM(left_flap_angle);
    int32_t right_flap_pwm = mapFlapAngleToPWM(right_flap_angle);

    // Build the 16-element command array:
    // First 12 entries are -1.
    // Positions 13 & 14 (indexes 12 & 13) for left/right wings.
    // Positions 15 & 16 (indexes 14 & 15) for left/right flaps.
    std_msgs::msg::Int32MultiArray command_msg;
    command_msg.data.resize(16, -1);
    command_msg.data[LEFT_WING] = left_wing_pwm;
    command_msg.data[RIGHT_WING] = right_wing_pwm;
    command_msg.data[LEFT_FLAP] = left_flap_pwm;
    command_msg.data[RIGHT_FLAP] = right_flap_pwm;

    // Publish the command message
    command_pub_->publish(command_msg);

    RCLCPP_INFO(this->get_logger(), "Published command: Wings[%d, %d] Flaps[%d, %d]",
                 left_wing_pwm, right_wing_pwm, left_flap_pwm, right_flap_pwm);
  }

  // Member variables
  rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr command_pub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr omega_m_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr A_m_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr A_d_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr alpha_d_sub_;

  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr theta_m_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr theta_d_sub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr start_stop_srv_;
  rclcpp::TimerBase::SharedPtr timer_;

  // Parameters
  double timer_frequency_; // Timer callback frequency (Hz)
  int angle_min_, angle_max_; // Wing angle range in degrees
  int pwm_min_, pwm_max_;     // PWM range
  int clamp_flap_min_, clamp_flap_max_; // Flap clamping range

  // Wing state variables
  double omega_m_;
  double A_m_;
  double A_d_;
  double alpha_d_;
  // Flap state variables (using the new two-topic approach)
  double theta_m_;
  double theta_d_;

  // Publishing enabled flag controlled via service call
  bool publishing_enabled_;

  rclcpp::Time start_time_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<CommandPublisher>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
