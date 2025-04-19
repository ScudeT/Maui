#include <chrono>
#include <cmath>
#include <memory>
#include <vector>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"

using namespace std::chrono_literals;

class PWMWavePublisher : public rclcpp::Node {
public:
  PWMWavePublisher() : Node("pwm_wave") {
    // Declare parameters.
    // 'addresses' is a list of indices to update with the sine value (default: {0}).
    // 'ignore' is a list of indices to override with -1 (default: empty).
    // 'center' is the center count value of the waveform.
    // 'amplitude' is the peak deviation from the center.
    // 'wave_freq' is the frequency (Hz) of the sine wave.
    //
    // The default center is set to the count corresponding to 1520 μs,
    // assuming a PWM period of 5000 μs (200 Hz) and full range 0–65535:
    //   1520 μs * (65535 / 5000) ≈ 19928.
    // The default amplitude is chosen so that the pulse varies roughly
    // from 920 μs to 2120 μs (~600 μs offset from center), which is about:
    //   600 μs * (65535 / 5000) ≈ 7864.
    this->declare_parameter<std::vector<int64_t>>("addresses", std::vector<int64_t>{0});
    this->declare_parameter<std::vector<int64_t>>("ignore", std::vector<int64_t>{});
    this->declare_parameter<int64_t>("center", 19928);
    this->declare_parameter<int64_t>("amplitude", 7864);
    this->declare_parameter<double>("wave_freq", 0.3);

    // Retrieve parameters.
    addresses_ = this->get_parameter("addresses").as_integer_array();
    ignore_ = this->get_parameter("ignore").as_integer_array();
    center_ = this->get_parameter("center").as_int();
    amplitude_ = this->get_parameter("amplitude").as_int();
    wave_freq_ = this->get_parameter("wave_freq").as_double();

    // Validate that each index in both lists is within [0, 15].
    for (int addr : addresses_) {
      if (addr < 0 || addr >= 16) {
        RCLCPP_ERROR(this->get_logger(), "Each address in 'addresses' must be between 0 and 15. Invalid address: %d", addr);
        rclcpp::shutdown();
        return;
      }
    }
    for (int addr : ignore_) {
      if (addr < 0 || addr >= 16) {
        RCLCPP_ERROR(this->get_logger(), "Each address in 'ignore' must be between 0 and 15. Invalid address: %d", addr);
        rclcpp::shutdown();
        return;
      }
    }

    // Create a publisher on the topic "pwm_wave_topic".
    publisher_ = this->create_publisher<std_msgs::msg::Int32MultiArray>("command", 10);
    start_time_ = this->now();

    // Set up a timer callback (every 20ms) to update the multiarray.
    timer_ = this->create_wall_timer(20ms, std::bind(&PWMWavePublisher::timer_callback, this));
  }

private:
  void timer_callback() {
    // Calculate elapsed time in seconds.
    auto current_time = this->now();
    double elapsed = (current_time - start_time_).seconds();

    // Compute the sine value using center and amplitude.
    // value = center + amplitude * sin(2*pi*wave_freq*t)
    double sine_val = center_ + amplitude_ * std::sin(2.0 * M_PI * wave_freq_ * elapsed);

    // Prepare the multiarray message with 16 elements, initialized to 0.
    std_msgs::msg::Int32MultiArray msg;
    msg.data.resize(16, 0);

    // Set the sine value on the specified addresses.
    for (int addr : addresses_) {
      msg.data[addr] = static_cast<int64_t>(sine_val);
    }

    // For addresses to ignore, override with -1.
    for (int addr : ignore_) {
      msg.data[addr] = -1;
    }

    // Publish the message.
    publisher_->publish(msg);
  }

  rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Time start_time_;
  std::vector<int64_t> addresses_;
  std::vector<int64_t> ignore_;
  int center_;
  int amplitude_;
  double wave_freq_;
};

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PWMWavePublisher>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
