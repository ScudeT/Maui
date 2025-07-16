#include <memory>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "nav_msgs/msg/odometry.hpp"

#define DEG2RAD(x) ((x) * M_PI / 180.0)

class GpsOdometryNode : public rclcpp::Node {
public:
  GpsOdometryNode() : Node("gps_odometry_node"), datum_initialized_(false) {
    gps_sub_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
      "/gps_data", 10,
      std::bind(&GpsOdometryNode::gps_callback, this, std::placeholders::_1)
    );

    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/myodometry/gps", 10);

    // Ellissoide WGS84
    a_ = 6378137.0;                  // raggio equatoriale
    f_ = 1.0 / 298.257223563;        // appiattimento
    k0_ = 0.9996;                    // scala UTM
    zone_ = 32;                      // zona UTM per Milano
    lambda0_ = DEG2RAD(3 + zone_ * 6 - 183);  // meridiano centrale
  }

private:
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;

  bool datum_initialized_;
  double x0_, y0_;

  // Costanti WGS84
  double a_, f_, k0_, lambda0_;
  int zone_;

  void gps_callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg) {
    if (msg->status.status < sensor_msgs::msg::NavSatStatus::STATUS_FIX) return;

    double lat = DEG2RAD(msg->latitude);
    double lon = DEG2RAD(msg->longitude);

    // Proiezione semplificata UTM
    double e = std::sqrt(f_ * (2 - f_));
    double N = a_ / std::sqrt(1 - e * e * std::sin(lat) * std::sin(lat));
    double T = std::tan(lat) * std::tan(lat);
    double C = (e * e) / (1 - e * e) * std::cos(lat) * std::cos(lat);
    double A = std::cos(lat) * (lon - lambda0_);

    double M = a_ * (
      (1 - e * e / 4 - 3 * e * e * e * e / 64 - 5 * e * e * e * e * e * e / 256) * lat
      - (3 * e * e / 8 + 3 * e * e * e * e / 32 + 45 * e * e * e * e * e * e / 1024) * std::sin(2 * lat)
      + (15 * e * e * e * e / 256 + 45 * e * e * e * e * e * e / 1024) * std::sin(4 * lat)
      - (35 * e * e * e * e * e * e / 3072) * std::sin(6 * lat)
    );

    double x = k0_ * N * (A + (1 - T + C) * std::pow(A,3)/6 + (5 - 18*T + T*T + 72*C - 58*(e*e)/(1-e*e)) * std::pow(A,5)/120);
    double y = k0_ * (M);

    if (!datum_initialized_) {
      x0_ = x;
      y0_ = y;
      datum_initialized_ = true;
      RCLCPP_INFO(this->get_logger(), "Datum initialized at lat %.6f, lon %.6f", msg->latitude, msg->longitude);
    }

    nav_msgs::msg::Odometry odom_msg;
    odom_msg.header.stamp = msg->header.stamp;
    odom_msg.header.frame_id = "map";
    odom_msg.child_frame_id = "gps_link";

    odom_msg.pose.pose.position.x = x - x0_;
    odom_msg.pose.pose.position.y = y - y0_;
    odom_msg.pose.pose.position.z = 0.0;

    odom_msg.pose.pose.orientation.w = 1.0; // no heading info

    // Covarianza fissa diagonale
    for (int i = 0; i < 36; ++i) odom_msg.pose.covariance[i] = 0.0;
    odom_msg.pose.covariance[0] = 0.1;
    odom_msg.pose.covariance[7] = 0.1;
    odom_msg.pose.covariance[35] = 9999.0;  // no orientation

    odom_pub_->publish(odom_msg);
  }
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GpsOdometryNode>());
  rclcpp::shutdown();
  return 0;
}