#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/fluid_pressure.hpp"
#include "std_msgs/msg/float64.hpp"
#include "MS5837.h"

class MS5837Node : public rclcpp::Node {
public:
    MS5837Node() : Node("ms5837_node"), sensor() {
        RCLCPP_INFO(this->get_logger(), "Initializing MS5837 Node...");
        
        // Initialize the MS5837 sensor
        if (!sensor.init("/dev/i2c-1")) {
            RCLCPP_ERROR(this->get_logger(), "Failed to initialize MS5837 sensor");
            rclcpp::shutdown();
            return;
        }
        RCLCPP_INFO(this->get_logger(), "MS5837 sensor initialized successfully");

        // Set fluid density for seawater (default is 1029 kg/m^3)
        sensor.setFluidDensity(1029);
        sensor.setModel(MS5837::MS5837_02BA);
        RCLCPP_INFO(this->get_logger(), "Fluid density set to 1029 kg/m^3 and sensor model set to MS5837_02BA");

        // Publishers
        fluid_pressure_pub_ = this->create_publisher<sensor_msgs::msg::FluidPressure>("/ms5837/fluid_pressure", 10);
        depth_pub_ = this->create_publisher<std_msgs::msg::Float64>("/ms5837/depth", 10);
        height_pub_ = this->create_publisher<std_msgs::msg::Float64>("/ms5837/height", 10);
        RCLCPP_INFO(this->get_logger(), "Publishers for fluid pressure, depth, and height topics created");

        // Timer to periodically read from the sensor and publish data
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&MS5837Node::timerCallback, this)
        );
        RCLCPP_INFO(this->get_logger(), "Timer set to publish data every 100 milliseconds");
    }

private:
    void timerCallback() {
        // Read sensor data
        sensor.readSensor();
        RCLCPP_INFO(this->get_logger(), "Sensor data read successfully");

        // Publish fluid pressure
        auto pressure_msg = sensor_msgs::msg::FluidPressure();
        pressure_msg.fluid_pressure = sensor.pressure(MS5837::mbar) * 100.0; // Convert mbar to Pascal
        pressure_msg.variance = 0.0;
        pressure_msg.header.frame_id = "bar_link";
        pressure_msg.header.stamp = this->get_clock()->now();
        fluid_pressure_pub_->publish(pressure_msg);
        RCLCPP_INFO(this->get_logger(), "Published fluid pressure: %.2f Pa", pressure_msg.fluid_pressure);

        // Publish depth
        auto depth_msg = std_msgs::msg::Float64();
        depth_msg.data = sensor.depth();
        depth_pub_->publish(depth_msg);
        RCLCPP_INFO(this->get_logger(), "Published depth: %.2f m", depth_msg.data);

        // Publish height
        auto height_msg = std_msgs::msg::Float64();
        height_msg.data = sensor.altitude();
        height_pub_->publish(height_msg);
        RCLCPP_INFO(this->get_logger(), "Published height: %.2f m", height_msg.data);
    }

    MS5837 sensor;
    rclcpp::Publisher<sensor_msgs::msg::FluidPressure>::SharedPtr fluid_pressure_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr depth_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr height_pub_;
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
