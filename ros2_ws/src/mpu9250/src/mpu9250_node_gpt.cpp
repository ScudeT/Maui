#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/magnetic_field.hpp"
#include "std_msgs/msg/float64.hpp"

#include "Wire.hpp"
#include "QuaternionFilter.hpp"
#include "MPU9250.hpp"

static constexpr double DEG2RAD = M_PI / 180.0;

class ImuNode : public rclcpp::Node {
public:
  ImuNode()
  : Node("mpu9250_node")
  {
    // Declare parameters with defaults
    this->declare_parameter<std::string>("frame_id", "base_link");
    this->declare_parameter<double>("freq", 50.0);
    this->declare_parameter<int>("filter", static_cast<int>(QuatFilterSel::MADGWICK));
    this->declare_parameter<int>("accel_fs_sel", static_cast<int>(ACCEL_FS_SEL::A2G));
    this->declare_parameter<int>("gyro_fs_sel", static_cast<int>(GYRO_FS_SEL::G250DPS));
    this->declare_parameter<int>("mag_output_bits", static_cast<int>(MAG_OUTPUT_BITS::M16BITS));
    this->declare_parameter<int>("fifo_sample_rate", static_cast<int>(FIFO_SAMPLE_RATE::SMPL_200HZ));
    this->declare_parameter<int>("gyro_dlpf_cfg", static_cast<int>(GYRO_DLPF_CFG::DLPF_41HZ));
    this->declare_parameter<int>("accel_dlpf_cfg", static_cast<int>(ACCEL_DLPF_CFG::DLPF_45HZ));
    this->declare_parameter<double>("g", 9.81);
    this->declare_parameter<double>("acc_cov", 0.01);
    this->declare_parameter<double>("w_cov", 0.01);
    this->declare_parameter<double>("rpy_cov", 0.0003);
    this->declare_parameter<std::vector<double>>("mag_bias", std::vector<double>{0.0, 0.0, 0.0});

    // Retrieve parameters
    frame_id_    = this->get_parameter("frame_id").as_string();
    double freq  = this->get_parameter("freq").as_double();
    int filter   = this->get_parameter("filter").as_int();
    int afs      = this->get_parameter("accel_fs_sel").as_int();
    int gfs      = this->get_parameter("gyro_fs_sel").as_int();
    int mbits    = this->get_parameter("mag_output_bits").as_int();
    int fsr      = this->get_parameter("fifo_sample_rate").as_int();
    int gdlpf    = this->get_parameter("gyro_dlpf_cfg").as_int();
    int adlpf    = this->get_parameter("accel_dlpf_cfg").as_int();
    g_           = this->get_parameter("g").as_double();
    acc_cov_     = this->get_parameter("acc_cov").as_double();
    w_cov_       = this->get_parameter("w_cov").as_double();
    rpy_cov_     = this->get_parameter("rpy_cov").as_double();
    auto mag_bias = this->get_parameter("mag_bias").as_double_array();

    // Validate update rate
    if (freq <= 0.0) {
      RCLCPP_ERROR(get_logger(), "Parameter 'freq' must be > 0");
      rclcpp::shutdown();
      return;
    }
    auto period = std::chrono::milliseconds(static_cast<int>(1000.0 / freq));

    // Map settings
    settings_.accel_fs_sel     = static_cast<ACCEL_FS_SEL>(afs);
    settings_.gyro_fs_sel      = static_cast<GYRO_FS_SEL>(gfs);
    settings_.mag_output_bits  = static_cast<MAG_OUTPUT_BITS>(mbits);
    settings_.fifo_sample_rate = static_cast<FIFO_SAMPLE_RATE>(fsr);
    settings_.gyro_dlpf_cfg    = static_cast<GYRO_DLPF_CFG>(gdlpf);
    settings_.accel_dlpf_cfg   = static_cast<ACCEL_DLPF_CFG>(adlpf);

    // Publishers
    imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("imu/data", 10);
    temp_pub_= this->create_publisher<std_msgs::msg::Float64>("imu/temperature", 10);
    mag_pub_ = this->create_publisher<sensor_msgs::msg::MagneticField>("imu/mag", 10);

    // I2C and sensor init
    i2c_ = std::make_shared<I2CHandler>('/' + std::string("dev/i2c-1"));
    imu_ = std::make_shared<MPU9250>(
      this->get_logger(),
      MPU9250::MPU9250_DEFAULT_ADDRESS,
      settings_,
      *i2c_
    );
    if (!imu_->isConnected()) {
      RCLCPP_ERROR(get_logger(), "Cannot connect to MPU9250");
      rclcpp::shutdown(); return;
    }

    // Apply user mag bias & calibrate
    imu_->setMagBias(mag_bias[0], mag_bias[1], mag_bias[2]);
    imu_->calibrateMag();

    // Select filter and reset scale
    imu_->select_filter(static_cast<QuatFilterSel>(filter));
    imu_->setMagScale(1.0f, 1.0f, 1.0f);

    // Timer
    timer_ = this->create_wall_timer(period,
      std::bind(&ImuNode::onTimer, this)
    );
    RCLCPP_INFO(get_logger(), "MPU9250 node initialized at %.1f Hz", freq);
  }

private:
  void onTimer() {
    if (!imu_->update()) {
      RCLCPP_WARN(get_logger(), "IMU update failed");
      return;
    }
    auto t = this->now();

    // IMU message
    sensor_msgs::msg::Imu imu_msg;
    imu_msg.header.stamp = t;
    imu_msg.header.frame_id = frame_id_;
    imu_msg.orientation.x = imu_->getQuaternionX();
    imu_msg.orientation.y = imu_->getQuaternionY();
    imu_msg.orientation.z = imu_->getQuaternionZ();
    imu_msg.orientation.w = imu_->getQuaternionW();
    imu_msg.orientation_covariance = {{rpy_cov_,0,0,0,rpy_cov_,0,0,0,rpy_cov_}};
    imu_msg.linear_acceleration.x = imu_->getAccX()*g_;
    imu_msg.linear_acceleration.y = imu_->getAccY()*g_;
    imu_msg.linear_acceleration.z = imu_->getAccZ()*g_;
    imu_msg.linear_acceleration_covariance = {{acc_cov_,0,0,0,acc_cov_,0,0,0,acc_cov_}};
    imu_msg.angular_velocity.x = imu_->getGyroX()*DEG2RAD;
    imu_msg.angular_velocity.y = imu_->getGyroY()*DEG2RAD;
    imu_msg.angular_velocity.z = imu_->getGyroZ()*DEG2RAD;
    imu_msg.angular_velocity_covariance = {{w_cov_,0,0,0,w_cov_,0,0,0,w_cov_}};
    imu_pub_->publish(imu_msg);

    // Temperature
    std_msgs::msg::Float64 temp_msg;
    temp_msg.data = imu_->getTemperature();
    temp_pub_->publish(temp_msg);

    // Magnetic field
    sensor_msgs::msg::MagneticField mag_msg;
    mag_msg.header.stamp = t;
    mag_msg.header.frame_id = frame_id_;
    mag_msg.magnetic_field.x = imu_->getMagX();
    mag_msg.magnetic_field.y = imu_->getMagY();
    mag_msg.magnetic_field.z = imu_->getMagZ();
    mag_msg.magnetic_field_covariance = {{0,0,0,0,0,0,0,0,0}};
    mag_pub_->publish(mag_msg);
  }

  // Members
  std::string frame_id_;
  double g_, acc_cov_, w_cov_, rpy_cov_;
  MPU9250Setting settings_;
  std::shared_ptr<I2CHandler> i2c_;
  std::shared_ptr<MPU9250> imu_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr temp_pub_;
  rclcpp::Publisher<sensor_msgs::msg::MagneticField>::SharedPtr mag_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ImuNode>());
  rclcpp::shutdown();
  return 0;
}
