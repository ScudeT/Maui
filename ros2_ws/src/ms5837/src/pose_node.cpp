#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "MS5837.h"

class MS5837Node : public rclcpp::Node {
public:
    MS5837Node() : Node("pose_node"), sensor() {
        RCLCPP_INFO(this->get_logger(), "Initializing MS5837 Node...");
        
        // Initialize the MS5837 sensor
        if (!sensor.init("/dev/i2c-1")) {
            RCLCPP_ERROR(this->get_logger(), "Failed to initialize MS5837 sensor");
            rclcpp::shutdown();
            return;
        }
        RCLCPP_INFO(this->get_logger(), "MS5837 sensor initialized successfully");

        // Set fluid density for seawater (default is 1029 kg/m^3)
        sensor.setFluidDensity(1000);
        sensor.setModel(MS5837::MS5837_02BA);
        RCLCPP_INFO(this->get_logger(), "Fluid density set to 1029 kg/m^3 and sensor model set to MS5837_02BA");

        this->declare_parameter<double>("offset", 0.0);
        offset_ = this->get_parameter("offset").as_double();

        this->declare_parameter<double>("variance", 0.0001);
        variance_ = this->get_parameter("variance").as_double();


        // Publishers
        pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/ms5837/pose", 10);
        RCLCPP_INFO(this->get_logger(), "Publishers for pose with covariance from depth");

        // Timer to periodically read from the sensor and publish data
        this->declare_parameter<double>("freq", 50.0); // Default: 50 Hz
        // Declare parameters with default values.
        this->declare_parameter<std::string>("frame_id", "base_link");

        frame_id_ = this->get_parameter("frame_id").as_string();
        
        // Retrieve the update_rate parameter.
        double update_rate = this->get_parameter("freq").as_double();
        if (update_rate <= 0.0) {
            RCLCPP_ERROR(this->get_logger(), "update_rate must be > 0. Shutting down node.");
            rclcpp::shutdown();
            return;
        }
        // Convert Hz to milliseconds period.
        int period_ms = static_cast<int>(1000.0 / update_rate);

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(period_ms),
            std::bind(&MS5837Node::timerCallback, this)
        );
        // Initialize covariance matrix (example values)
        // For a 6x6 covariance:
        // [x, y, z, roll, pitch, yaw]
        // You can tune these values as needed.
        for (size_t i = 0; i < 36; ++i)
        {
        pose_covariance_[i] = 0.0;
        }
        // Example: give some uncertainty in z position
        pose_covariance_[2 * 6 + 2] = variance_; // z variance = 0.05 m^2 (example)
        // Add small uncertainties in x,y just for completeness
        pose_covariance_[0 * 6 + 0] = 1e6; // x variance
        pose_covariance_[1 * 6 + 1] = 1e6; // y variance
        // Orientation uncertainty (roll, pitch, yaw)
        pose_covariance_[3 * 6 + 3] = 1e6;
        pose_covariance_[4 * 6 + 4] = 1e6;
        pose_covariance_[5 * 6 + 5] = 1e6;
    }

private:
    void timerCallback() {
        // Read sensor data
        sensor.readSensor();
        RCLCPP_INFO(this->get_logger(), "Sensor data read successfully");

        // Publish depth
        float depth = sensor.depth();

        // Create PoseWithCovarianceStamped
        geometry_msgs::msg::PoseWithCovarianceStamped pose_msg;
        pose_msg.header.stamp = this->now();
        pose_msg.header.frame_id = frame_id_;

        // Set the pose
        pose_msg.pose.pose.position.x = 0.0;
        pose_msg.pose.pose.position.y = 0.0;
        pose_msg.pose.pose.position.z = depth + offset_;

        // Assuming no specific orientation known, set quaternion for no rotation
        pose_msg.pose.pose.orientation.x = 0.0;
        pose_msg.pose.pose.orientation.y = 0.0;
        pose_msg.pose.pose.orientation.z = 0.0;
        pose_msg.pose.pose.orientation.w = 1.0;

        // Copy covariance
        for (size_t i = 0; i < 36; ++i)
        {
        pose_msg.pose.covariance[i] = pose_covariance_[i];
        }

        // Publish the message
        pose_pub_->publish(pose_msg);
        RCLCPP_INFO(this->get_logger(), "Published depth: %.2f m", depth);


    }

    MS5837 sensor;
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_pub_;

    std::string frame_id_;
    double pose_covariance_[36];

    double offset_;
    double variance_;

    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Starting MS5837 Node...");
    rclcpp::spin(std::make_shared<MS5837Node>());
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Shutting down MS5837 Node...");
    rclcpp::shutdown();
    return 0;
}
