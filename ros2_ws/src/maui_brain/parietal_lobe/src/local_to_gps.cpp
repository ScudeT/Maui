#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <cmath>

class GpsFilteredNode : public rclcpp::Node {
public:
  GpsFilteredNode() : Node("gps_filtered_node") {
    gps_pub_ = this->create_publisher<sensor_msgs::msg::NavSatFix>("gps/filtered", 10);

    odom_filtered_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odometry/filtered", 10,
      std::bind(&GpsFilteredNode::odom_filtered_callback, this, std::placeholders::_1)
    );

    gps_origin_sub_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
      "/gps_data", 10,
      std::bind(&GpsFilteredNode::gps_origin_callback, this, std::placeholders::_1)
    );

    origin_set_ = false;
  }

private:
  void gps_origin_callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg) {
    if (!origin_set_) {
      origin_lat_ = msg->latitude;
      origin_lon_ = msg->longitude;
      origin_alt_ = msg->altitude;
      origin_lat_rad_ = origin_lat_ * M_PI / 180.0;
      origin_set_ = true;

      RCLCPP_INFO(this->get_logger(), "Origin set from GPS fix: lat=%.6f, lon=%.6f", origin_lat_, origin_lon_);
    }
  }

  void odom_filtered_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    if (!origin_set_) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                           "Waiting for GPS fix to set origin...");
      return;
    }

    double x = msg->pose.pose.position.x;
    double y = msg->pose.pose.position.y;
    //double z = msg->pose.pose.position.z;

    const double R = 6378137.0;  // raggio medio terrestre in metri
    double delta_lat = y / R;
    double delta_lon = x / (R * cos(origin_lat_rad_));

    double lat = origin_lat_ + delta_lat * (180.0 / M_PI);
    double lon = origin_lon_ + delta_lon * (180.0 / M_PI);
    double alt = origin_alt_;  // opzionale

    sensor_msgs::msg::NavSatFix fix;
    fix.header.stamp = msg->header.stamp;
    fix.header.frame_id = "base_link";
    fix.latitude = lat;
    fix.longitude = lon;
    fix.altitude = alt;
    fix.status.status = sensor_msgs::msg::NavSatStatus::STATUS_FIX;
    fix.status.service = sensor_msgs::msg::NavSatStatus::SERVICE_GPS;
    fix.position_covariance_type = sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN;
    fix.position_covariance = {
      0.1, 0.0, 0.0,
      0.0, 0.1, 0.0,
      0.0, 0.0, 0.1
    };

    gps_pub_->publish(fix);
  }

  rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr gps_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_filtered_sub_;
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_origin_sub_;

  bool origin_set_;
  double origin_lat_, origin_lon_, origin_alt_, origin_lat_rad_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GpsFilteredNode>());
  rclcpp::shutdown();
  return 0;
}