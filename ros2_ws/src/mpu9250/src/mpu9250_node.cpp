#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/magnetic_field.hpp"
#include "std_msgs/msg/float64.hpp"
#include "MPU9250.h"

#define DEG2RAD 3.14/180

class ImuNode : public rclcpp::Node {
public:
    ImuNode() : Node("mpu9250_node") {
    // Declare parameters with default values.
    this->declare_parameter<std::string>("frame_id", "base_link");

    frame_id_ = this->get_parameter("frame_id").as_string();

    // Here, update_rate is in Hz (times per second).
    this->declare_parameter<double>("freq", 50.0); // Default: 50 Hz
    this->declare_parameter<int>("filter", static_cast<int>(QuatFilterSel::MAHONY));
    this->declare_parameter<int>("accel_fs_sel", static_cast<int>(ACCEL_FS_SEL::A4G));
    this->declare_parameter<int>("gyro_fs_sel", static_cast<int>(GYRO_FS_SEL::G500DPS));
    this->declare_parameter<int>("mag_output_bits", static_cast<int>(MAG_OUTPUT_BITS::M14BITS));
    this->declare_parameter<int>("fifo_sample_rate", static_cast<int>(FIFO_SAMPLE_RATE::SMPL_500HZ));
    this->declare_parameter<int>("gyro_dlpf_cfg", static_cast<int>(GYRO_DLPF_CFG::DLPF_41HZ));
    this->declare_parameter<int>("accel_dlpf_cfg", static_cast<int>(ACCEL_DLPF_CFG::DLPF_45HZ));
    this->declare_parameter<double>("g", 9.81);
    this->declare_parameter<double>("acc_cov", 0.01);
    this->declare_parameter<double>("w_cov", 0.01);
    this->declare_parameter<double>("rpy_cov", 0.0003);
    this->declare_parameter<std::vector<double>>("mag_bias", std::vector<double>{500.0, 1500.0, 1250.0});
    
    // Retrieve the update_rate parameter.
    double update_rate = this->get_parameter("freq").as_double();
    if (update_rate <= 0.0) {
        RCLCPP_ERROR(this->get_logger(), "update_rate must be > 0. Shutting down node.");
        rclcpp::shutdown();
        return;
    }
    // Convert Hz to milliseconds period.
    int period_ms = static_cast<int>(1000.0 / update_rate);

    // Retrieve other parameter values.
    int filter_val = this->get_parameter("filter").as_int();
    int accel_fs_sel_val = this->get_parameter("accel_fs_sel").as_int();
    int gyro_fs_sel_val = this->get_parameter("gyro_fs_sel").as_int();
    int mag_output_bits_val = this->get_parameter("mag_output_bits").as_int();
    int fifo_sample_rate_val = this->get_parameter("fifo_sample_rate").as_int();
    int gyro_dlpf_cfg_val = this->get_parameter("gyro_dlpf_cfg").as_int();
    int accel_dlpf_cfg_val = this->get_parameter("accel_dlpf_cfg").as_int();
    std::vector<double> mag_bias = this->get_parameter("mag_bias").as_double_array();


    // Create publishers.
    imu_publisher_ = this->create_publisher<sensor_msgs::msg::Imu>("imu/data", 10);
    temperature_publisher_ = this->create_publisher<std_msgs::msg::Float64>("imu/temperature", 10);
    mag_publisher_ = this->create_publisher<sensor_msgs::msg::MagneticField>("imu/mag", 10);

    // Setup timer using the computed period.
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(period_ms),
        std::bind(&ImuNode::publishImuData, this));

    // Initialize I2C handler and configure the IMU.
    i2cBus_ = std::make_shared<I2CHandler>("/dev/i2c-1");

    // Map the parameter values to the corresponding enum types.
    settings_.accel_fs_sel = static_cast<ACCEL_FS_SEL>(accel_fs_sel_val);
    settings_.gyro_fs_sel = static_cast<GYRO_FS_SEL>(gyro_fs_sel_val);
    settings_.mag_output_bits = static_cast<MAG_OUTPUT_BITS>(mag_output_bits_val);
    settings_.fifo_sample_rate = static_cast<FIFO_SAMPLE_RATE>(fifo_sample_rate_val);
    settings_.gyro_dlpf_cfg = static_cast<GYRO_DLPF_CFG>(gyro_dlpf_cfg_val);
    settings_.accel_dlpf_cfg = static_cast<ACCEL_DLPF_CFG>(accel_dlpf_cfg_val);

    imu_ = std::make_shared<MPU9250>(0x68, settings_, *i2cBus_);

    if (!imu_->isConnected()) {
        RCLCPP_ERROR(this->get_logger(), "Failed to connect to MPU9250!");
        rclcpp::shutdown();
        return;
    }
    imu_->setMagBias(mag_bias[0], mag_bias[1], mag_bias[2]);
    RCLCPP_WARN(this->get_logger(), "Starting calibration for MPU9250...");
    imu_->calibrateAccelGyro();
    //imu_->calibrateMag();
    RCLCPP_WARN(this->get_logger(), "Calibration complete. Configuring MPU9250...");

    // Use the enum value for filter selection directly.
    QuatFilterSel filter_sel = static_cast<QuatFilterSel>(filter_val);
    imu_->selectFilter(filter_sel);

    // Configure magnetometer scale.
    imu_->setMagScale(1, 1, 1); // Data in mG (milli Gauss)

    g_ =        this->get_parameter("g").as_double();
    acc_cov_ =  this->get_parameter("acc_cov").as_double();
    w_cov_ =    this->get_parameter("w_cov").as_double();
    rpy_cov_ =  this->get_parameter("rpy_cov").as_double();

    RCLCPP_WARN(this->get_logger(), "MPU9250 initialized and calibrated.");
}

private:
    void publishImuData() {
        if (imu_->update()) {
            // Publish IMU data
            sensor_msgs::msg::Imu imu_msg;
            imu_msg.header.stamp = this->now();
            imu_msg.header.frame_id = frame_id_;

            // Populate orientation using quaternion
            imu_msg.orientation.x = imu_->getQuaternionX();
            imu_msg.orientation.y = imu_->getQuaternionY();
            imu_msg.orientation.z = imu_->getQuaternionZ();
            imu_msg.orientation.w = imu_->getQuaternionW();

            // No covariance for orientation in this example
            imu_msg.orientation_covariance = {rpy_cov_, 0.0, 0.0, 0.0, rpy_cov_, 0.0, 0.0, 0.0, rpy_cov_};

            // Populate linear acceleration
            imu_msg.linear_acceleration.x = imu_->getAccX()*g_;
            imu_msg.linear_acceleration.y = imu_->getAccY()*g_;
            imu_msg.linear_acceleration.z = imu_->getAccZ()*g_;

            // No covariance for linear acceleration in this example
            imu_msg.linear_acceleration_covariance = {acc_cov_, 0.0, 0.0, 0.0, acc_cov_, 0.0, 0.0, 0.0, acc_cov_};

            // Populate angular velocity
            imu_msg.angular_velocity.x = imu_->getGyroX()*DEG2RAD;
            imu_msg.angular_velocity.y = imu_->getGyroY()*DEG2RAD;
            imu_msg.angular_velocity.z = imu_->getGyroZ()*DEG2RAD;

            // No covariance for angular velocity in this example
            imu_msg.angular_velocity_covariance = {w_cov_, 0.0, 0.0, 0.0, w_cov_, 0.0, 0.0, 0.0, w_cov_};

            imu_publisher_->publish(imu_msg);

            // Publish temperature data
            std_msgs::msg::Float64 temp_msg;
            temp_msg.data = imu_->getTemperature();
            temperature_publisher_->publish(temp_msg);

            // Publish magnetometer data
            sensor_msgs::msg::MagneticField mag_msg;
            mag_msg.header.stamp = this->get_clock()->now();
            mag_msg.header.frame_id = "base_link";

            mag_msg.magnetic_field.x = imu_->getMagY();
            mag_msg.magnetic_field.y = imu_->getMagX();
            mag_msg.magnetic_field.z = - imu_->getMagZ();

            // Covariance is unknown in this example, so set to zero
            mag_msg.magnetic_field_covariance = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

            mag_publisher_->publish(mag_msg);

            RCLCPP_INFO(this->get_logger(), "Published IMU, Temperature, and Magnetometer data.");
        } else {
            RCLCPP_WARN(this->get_logger(), "Failed to update IMU data.");
        }
    }

    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_publisher_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr temperature_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::MagneticField>::SharedPtr mag_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::string frame_id_;

    std::shared_ptr<I2CHandler> i2cBus_;
    MPU9250Setting settings_;
    std::shared_ptr<MPU9250> imu_;
    float g_;
    float acc_cov_;
    float w_cov_;
    float rpy_cov_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ImuNode>());
    rclcpp::shutdown();
    return 0;
}
