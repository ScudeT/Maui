#include <memory>
#include <mutex>
#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include <tf2/LinearMath/Quaternion.h>
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

using namespace std::chrono_literals;

class OdometryEstimator : public rclcpp::Node
{
public:
  OdometryEstimator() : Node("odometry_estimator")
  {
    // Parametri
    this->declare_parameter<double>("publish_rate", 30.0);
    publish_rate_ = this->get_parameter("publish_rate").as_double();
    dt_ = 1.0 / publish_rate_;
    auto period = std::chrono::duration<double>(1.0 / publish_rate_);


    this->declare_parameter<std::string>("header_frame", "odom");
    header_frame_ = this->get_parameter("header_frame").as_string();

    this->declare_parameter<std::string>("child_frame", "base_link");
    child_frame_ = this->get_parameter("child_frame").as_string();

    // Subscriber
    imu_subscriber_ = this->create_subscription<sensor_msgs::msg::Imu>(
      "/imu/data", 10,
      std::bind(&OdometryEstimator::imu_callback, this, std::placeholders::_1));

    pose_subscriber_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "/ms5837/pose", 10,
      std::bind(&OdometryEstimator::pose_callback, this, std::placeholders::_1));

    // Publisher
    odom_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", 10);

    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

    timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::milliseconds>(period),
      std::bind(&OdometryEstimator::timer_callback, this));
    
    // Get Tasting gains
    double dt = 1.0 / publish_rate_;
    this->declare_parameter<double>("cutoff_freq", 10.0);
    double cutoff_freq = this->get_parameter("cutoff_freq").as_double();
    
    kf_ = - (M_PI * cutoff_freq * dt + 1.0)/ (M_PI * cutoff_freq * dt -1.0);
    kr_ = - (M_PI * cutoff_freq * dt)/ (M_PI * cutoff_freq * dt + 1.0);

    // Inizializza flag
    received_imu_ = false;
    received_pose_ = false;
    prev_gyro_set_ = false;

    RCLCPP_INFO(this->get_logger(), "Odometry Estimator node with fusion initialized.");
  }

private:
  // Callback IMU
  void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    last_imu_ = *msg;
    received_imu_ = true;
  }

  // Callback posizione barometrica
  void pose_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    last_pose_ = *msg;
    received_pose_ = true;
  }

  // Filtro sul gyro usando tasting con cutoff frequency
  geometry_msgs::msg::Vector3 filter_gyro(
    const geometry_msgs::msg::Vector3& raw_gyro)
  {
    // Converti in tf2::Vector3
    tf2::Vector3 raw_gyro_tf(raw_gyro.x, raw_gyro.y, raw_gyro.z);

    // Se prima chiamata: inizializza
    if (!prev_gyro_set_) {
      prev_filtered_gyro_ = raw_gyro_tf;
      prev_raw_gyro_ = raw_gyro_tf;
      prev_gyro_set_ = true;
      return raw_gyro;
    }

    tf2::Vector3 filtered;
    filtered = kf_ * prev_filtered_gyro_ + kr_ * (raw_gyro_tf + prev_raw_gyro_);

    prev_filtered_gyro_ = filtered;
    prev_raw_gyro_ = raw_gyro_tf;
    // Converti di nuovo in geometry_msgs::msg::Vector3
    geometry_msgs::msg::Vector3 filtered_msg;
    filtered_msg.x = filtered.getX();
    filtered_msg.y = filtered.getY();
    filtered_msg.z = filtered.getZ();

    return filtered_msg;
  }

  // Timer
  void timer_callback()
  {
    sensor_msgs::msg::Imu current_imu;
    geometry_msgs::msg::PoseWithCovarianceStamped current_pose;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (!received_imu_ || !received_pose_) {
        RCLCPP_INFO(this->get_logger(), "Waiting for both IMU and pose messages...");
        return;
      }
      current_imu = last_imu_;
      current_pose = last_pose_;
    }

    // Odometry msg
    nav_msgs::msg::Odometry odom_msg;
    odom_msg.header.stamp = this->now();
    odom_msg.header.frame_id = header_frame_;
    odom_msg.child_frame_id = child_frame_;

    // Posizione: sempre dal barometrico
    odom_msg.pose.pose.position = current_pose.pose.pose.position;
    odom_msg.pose.covariance = current_pose.pose.covariance;

    // Orientamento: IMU (corretto)
    tf2::Quaternion imu_quat, correction, corrected;
    tf2::fromMsg(current_imu.orientation, imu_quat);
    correction.setRPY(0.0, 0.0, 0.0);
    corrected = correction * imu_quat;
    odom_msg.pose.pose.orientation = tf2::toMsg(corrected);

    // Twist: velocità lineare nulla (o puoi calcolare la derivata della posizione)
    odom_msg.twist.twist.linear.x = 0.0;
    odom_msg.twist.twist.linear.y = 0.0;
    odom_msg.twist.twist.linear.z = 0.0;

    // Twist: **velocità angolari fuse**
    auto filtered_gyro = filter_gyro(current_imu.angular_velocity);
    odom_msg.twist.twist.angular.x = filtered_gyro.x;
    odom_msg.twist.twist.angular.y = filtered_gyro.y;
    odom_msg.twist.twist.angular.z = filtered_gyro.z;

    // Pubblica odom
    odom_publisher_->publish(odom_msg);

    // Pubblica anche il transform
    geometry_msgs::msg::TransformStamped transformStamped;
    transformStamped.header.stamp = this->now();
    transformStamped.header.frame_id = header_frame_;
    transformStamped.child_frame_id = child_frame_;
    transformStamped.transform.translation.x = odom_msg.pose.pose.position.x;
    transformStamped.transform.translation.y = odom_msg.pose.pose.position.y;
    transformStamped.transform.translation.z = odom_msg.pose.pose.position.z;
    transformStamped.transform.rotation = odom_msg.pose.pose.orientation;
    tf_broadcaster_->sendTransform(transformStamped);
  }

  // ---- VARIABILI ----
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscriber_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_subscriber_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::TimerBase::SharedPtr timer_;
  sensor_msgs::msg::Imu last_imu_;
  geometry_msgs::msg::PoseWithCovarianceStamped last_pose_;
  bool received_imu_{false};
  bool received_pose_{false};
  std::mutex mutex_;
  double publish_rate_;
  std::string header_frame_;
  std::string child_frame_;

  // Per il filtro gyro
  tf2::Vector3 prev_filtered_gyro_;
  tf2::Vector3 prev_raw_gyro_;
  bool prev_gyro_set_;

  double kf_;
  double kr_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<OdometryEstimator>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
