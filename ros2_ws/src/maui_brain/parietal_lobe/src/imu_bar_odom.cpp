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
    // Declare and get the publish_rate parameter (in Hz).
    this->declare_parameter<double>("publish_rate", 10.0);
    publish_rate_ = this->get_parameter("publish_rate").as_double();
    dt_ = 1.0 / publish_rate_;
    auto period = std::chrono::duration<double>(1.0 / publish_rate_);

    this->declare_parameter<double>("gyro_cutoff", 10.0);
    gyro_cutoff_ = this->get_parameter("gyro_cutoff").as_double();

    this->declare_parameter<std::string>("header_frame", "odom");
    header_frame_ = this->get_parameter("header_frame").as_string();

    this->declare_parameter<std::string>("child_frame", "base_link");
    child_frame_ = this->get_parameter("child_frame").as_string();

    // Subscribe to the IMU data topic.
    imu_subscriber_ = this->create_subscription<sensor_msgs::msg::Imu>(
      "/imu/data", 10,
      std::bind(&OdometryEstimator::imu_callback, this, std::placeholders::_1));

    // Subscribe to the ms5837 pose topic.
    pose_subscriber_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "/ms5837/pose", 10,
      std::bind(&OdometryEstimator::pose_callback, this, std::placeholders::_1));

    // Publisher for the odometry estimate.
    odom_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", 10);

    // Create a TransformBroadcaster for publishing the TF transform.
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

    // Timer callback to publish odometry and TF at the given rate.
    timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::milliseconds>(period),
      std::bind(&OdometryEstimator::timer_callback, this));

    RCLCPP_INFO(this->get_logger(), "Odometry Estimator node initialized.");
  }

private:
  // IMU callback: update the stored orientation.
  void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    last_imu_ = *msg;
    received_imu_ = true;
  }

  // Pose callback: update the stored position and covariance.
  void pose_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    last_pose_ = *msg;
    received_pose_ = true;
  }

  // Timer callback: combine the latest data and publish the odometry and TF transform.
  void timer_callback()
  {
    // Ensure we have received both sensor messages.
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

    // Create an odometry message.
    nav_msgs::msg::Odometry odom_msg;
    odom_msg.header.stamp = this->now();
    odom_msg.header.frame_id = header_frame_;
    odom_msg.child_frame_id = child_frame_;

    // Use the position (and covariance) from the ms5837 sensor.
    odom_msg.pose.pose.position = current_pose.pose.pose.position;
    odom_msg.pose.covariance = current_pose.pose.covariance;

    // Correct the IMU orientation by rotating it by pi around the x-axis.
    tf2::Quaternion imu_quat, correction, corrected;
    tf2::fromMsg(current_imu.orientation, imu_quat);
    correction.setRPY(M_PI, 0.0, 0.0);
    corrected = correction * imu_quat;
    odom_msg.pose.pose.orientation = tf2::toMsg(corrected);

    // (Optional) Set twist to zero. In practice, you might compute this based on sensor data.
    odom_msg.twist.twist.linear.x = 0.0;
    odom_msg.twist.twist.linear.y = 0.0;
    odom_msg.twist.twist.linear.z = 0.0;
    odom_msg.twist.twist.angular.x = current_imu.angular_velocity.x;
    odom_msg.twist.twist.angular.y = current_imu.angular_velocity.y;
    odom_msg.twist.twist.angular.z = current_imu.angular_velocity.z;

    // Publish the odometry message.
    odom_publisher_->publish(odom_msg);

    // Prepare and send the TF transform from odom to base_link.
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

  tf2::Vector3 filter_gyro()
  {
    double wx_raw = current_imu.angular_velocity.x;
    
  } 

  // Subscribers
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscriber_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_subscriber_;

  // Publisher for odometry.
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;

  // TF broadcaster.
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  // Timer for periodic publishing.
  rclcpp::TimerBase::SharedPtr timer_;

  // Variables to store the latest sensor data.
  sensor_msgs::msg::Imu last_imu_;
  geometry_msgs::msg::PoseWithCovarianceStamped last_pose_;
  bool received_imu_{false};
  bool received_pose_{false};

  // Mutex to guard shared data.
  std::mutex mutex_;

  // Publishing rate in Hz.
  double publish_rate_;
  std::string header_frame_;
  std::string child_frame_;

  // filter gyro data
  double dt_;
  double gyro_cutoff_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<OdometryEstimator>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
