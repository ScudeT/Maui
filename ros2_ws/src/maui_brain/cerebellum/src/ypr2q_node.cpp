#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <tf2/LinearMath/Quaternion.h>

#define DEG2RAD(x) ((x) * M_PI / 180.0)

class EulerToQuaternionNode : public rclcpp::Node {
public:
    EulerToQuaternionNode() : Node("euler_to_quaternion_node") {
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

        quat_pub_ = this->create_publisher<geometry_msgs::msg::Quaternion>(
            "q", 10);

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),  // 10 Hz
            std::bind(&EulerToQuaternionNode::timerCallback, this));
    }

private:
    void timerCallback() {
        tf2::Quaternion q;
        q.setRPY(roll_, pitch_, yaw_);
        q.normalize();

        geometry_msgs::msg::Quaternion msg;
        msg.x = q.x();
        msg.y = q.y();
        msg.z = q.z();
        msg.w = q.w();

        quat_pub_->publish(msg);
    }

    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr roll_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr pitch_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr yaw_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Quaternion>::SharedPtr quat_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    float roll_ = 0.0f;
    float pitch_ = 0.0f;
    float yaw_ = 0.0f;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<EulerToQuaternionNode>());
    rclcpp::shutdown();
    return 0;
}
