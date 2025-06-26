#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "std_msgs/msg/float32.hpp"
#include <cmath>

class SinusoidalDisturbanceNode : public rclcpp::Node
{
public:
    SinusoidalDisturbanceNode() : Node("sinusoidal_disturbance_node")
    {
        // Declare parameters
        this->declare_parameter("Dist_x_amp", 0.0);
        this->declare_parameter("Dist_y_amp", 0.0);
        this->declare_parameter("Dist_z_amp", 0.0);
        this->declare_parameter("frequency", 0.5);
        this->declare_parameter("sampling_rate", 1.0);

        // Get parameters
        amplitude_x_ = this->get_parameter("Dist_x_amp").as_double();
        amplitude_y_ = this->get_parameter("Dist_y_amp").as_double();
        amplitude_z_ = this->get_parameter("Dist_z_amp").as_double();

        frequency_ = this->get_parameter("frequency").as_double();
        sampling_rate_ = this->get_parameter("sampling_rate").as_double();

        // Create subscription and publishers
        input_subscriber_ = this->create_subscription<geometry_msgs::msg::Vector3>(
            "input_raw", 10, std::bind(&SinusoidalDisturbanceNode::inputCallback, this, std::placeholders::_1));

        x_publisher_ = this->create_publisher<std_msgs::msg::Float32>("output_x", 10);
        y_publisher_ = this->create_publisher<std_msgs::msg::Float32>("output_y", 10);
        z_publisher_ = this->create_publisher<std_msgs::msg::Float32>("output_z", 10);

        // Timer to update the disturbance at the given sampling rate
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(static_cast<int>(1000.0 / sampling_rate_)),
            std::bind(&SinusoidalDisturbanceNode::updateDisturbance, this));
    }

private:
    void inputCallback(const geometry_msgs::msg::Vector3::SharedPtr msg)
    {
        // Compute elapsed time in seconds
        auto current_time = this->now();
        double t = (current_time - start_time_).seconds();

        // Compute the wing motion using a sine function.
        double sine_val = sin(2 * M_PI * frequency_ * t);

        double disturbance_x = amplitude_x_ * sine_val;
        double disturbance_y = amplitude_y_ * sine_val;
        double disturbance_z = amplitude_z_ * sine_val; // Not used, but can be applied if needed

        float corrected_x = msg->x - disturbance_x;
        float corrected_y = msg->y - disturbance_y;
        float corrected_z = msg->z - disturbance_z; // No disturbance applied to Z

        // Publish the corrected values
        std_msgs::msg::Float32 x_msg;
        std_msgs::msg::Float32 y_msg;
        std_msgs::msg::Float32 z_msg;

        x_msg.data = corrected_x;
        y_msg.data = corrected_y;
        z_msg.data = corrected_z;

        x_publisher_->publish(x_msg);
        y_publisher_->publish(y_msg);
        z_publisher_->publish(z_msg);
    }

    void updateDisturbance()
    {
        amplitude_x_ = this->get_parameter("Dist_x_amp").as_double();
        amplitude_y_ = this->get_parameter("Dist_y_amp").as_double();
        amplitude_z_ = this->get_parameter("Dist_z_amp").as_double();
        frequency_ = this->get_parameter("frequency").as_double();
    }

    rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr input_subscriber_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr x_publisher_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr y_publisher_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr z_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Time start_time_ = this->now();

    double amplitude_x_;
    double amplitude_y_;
    double amplitude_z_;
    double frequency_;
    double sampling_rate_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SinusoidalDisturbanceNode>());
    rclcpp::shutdown();
    return 0;
}
