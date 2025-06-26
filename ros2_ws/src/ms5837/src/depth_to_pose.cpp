#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>

class DepthToPoseNode : public rclcpp::Node
{
public:
  DepthToPoseNode() : Node("depth_to_pose_node")
  {
    // Parameters
    this->declare_parameter<std::string>("frame_id", "base_link");
    this->declare_parameter<double>("offset", 0.0);

    frame_id_ = this->get_parameter("frame_id").as_string();
    offset_ = this->get_parameter("offset").as_double();

    depth_sub_ = this->create_subscription<std_msgs::msg::Float64>(
      "/ms5837/depth", 10, std::bind(&DepthToPoseNode::depthCallback, this, std::placeholders::_1));

    pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/ms5837/pose_est", 10);

    // Initialize covariance matrix (example values)
    // For a 6x6 covariance:
    // [x, y, z, roll, pitch, yaw]
    // You can tune these values as needed.
    for (size_t i = 0; i < 36; ++i)
    {
      pose_covariance_[i] = 0.0;
    }
    // Example: give some uncertainty in z position
    pose_covariance_[2 * 6 + 2] = 0.05; // z variance = 0.05 m^2 (example)
    // Add small uncertainties in x,y just for completeness
    pose_covariance_[0 * 6 + 0] = 0.01; // x variance
    pose_covariance_[1 * 6 + 1] = 0.01; // y variance
    // Orientation uncertainty (roll, pitch, yaw)
    pose_covariance_[3 * 6 + 3] = 0.001;
    pose_covariance_[4 * 6 + 4] = 0.001;
    pose_covariance_[5 * 6 + 5] = 0.001;
  }

private:
  void depthCallback(const std_msgs::msg::Float64::SharedPtr msg)
  {
    // Create PoseWithCovarianceStamped
    geometry_msgs::msg::PoseWithCovarianceStamped pose_msg;
    pose_msg.header.stamp = this->now();
    pose_msg.header.frame_id = frame_id_;

    // Set the pose
    pose_msg.pose.pose.position.x = 0.0;
    pose_msg.pose.pose.position.y = 0.0;
    pose_msg.pose.pose.position.z = msg->data - offset_;

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
  }

  std::string depth_topic_;
  std::string pose_topic_;
  std::string frame_id_;
  double offset_;
  double pose_covariance_[36];

  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr depth_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_pub_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<DepthToPoseNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
