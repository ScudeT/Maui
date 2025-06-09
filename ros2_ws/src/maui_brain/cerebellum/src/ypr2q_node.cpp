#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>

#define DEG2RAD(x) ((x) * M_PI / 180.0)

class EulerToPoseNode : public rclcpp::Node {
public:
    EulerToPoseNode() : Node("euler_to_pose_node") {
        roll_sub_ = this->create_subscription<std_msgs::msg::Float32>(
            "roll", 10, [this](const std_msgs::msg::Float32::SharedPtr msg) {
                roll_ = DEG2RAD(msg->data);
            });

        pitch_sub_ = this->create_subscription<std_msgs::msg::Float32>(
            "pitch", 10, [this](const std_msgs::msg::Float32::SharedPtr msg) {
                pitch_ = DEG2RAD(msg->data);
            });

        yaw_sub_ = this->create_subscription<std_msgs::msg::Float32>(
            "yaw", 10, [this](const std_msgs::msg::Float32::SharedPtr msg) {
                yaw_ = DEG2RAD(msg->data);
            });

        pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
            "q", 10);  // Still publishing on topic "q" but as Pose

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&EulerToPoseNode::timerCallback, this));
    }

private:
    void timerCallback() {
        tf2::Quaternion q;
        q.setRPY(roll_, pitch_, yaw_);
        q.normalize();

        geometry_msgs::msg::PoseStamped msg;
        msg.header.stamp = this->now();
        msg.header.frame_id = "base_link";  // Set frame_id as needed
        msg.pose.position.x = 0.0;
        msg.pose.position.y = 0.0;
        msg.pose.position.z = 0.0;
        msg.pose.orientation.x = q.x();
        msg.pose.orientation.y = q.y();
        msg.pose.orientation.z = q.z();
        msg.pose.orientation.w = q.w();

        pose_pub_->publish(msg);
    }

    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr roll_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr pitch_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr yaw_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    float roll_ = 0.0f;
    float pitch_ = 0.0f;
    float yaw_ = 0.0f;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<EulerToPoseNode>());
    rclcpp::shutdown();
    return 0;
}

