#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

class ImuTfTransformNode : public rclcpp::Node
{
public:
    ImuTfTransformNode()
    : Node("imu_tf_transform_node"),
      tf_buffer_(this->get_clock()),
      tf_listener_(tf_buffer_)
    {
        // Parameters: source and target frames (defaults)
        this->declare_parameter<std::string>("source_frame", "imu_link");
        this->declare_parameter<std::string>("target_frame", "imu_link_enu");
        source_frame_ = this->get_parameter("source_frame").as_string();
        target_frame_ = this->get_parameter("target_frame").as_string();

        // Wait for the static tf at construction
        bool got_tf = false;
        RCLCPP_INFO(this->get_logger(), "Waiting for static transform %s -> %s...", source_frame_.c_str(), target_frame_.c_str());
        for (int i = 0; i < 100; ++i) { // Wait up to 5 seconds
            try {
                auto tf = tf_buffer_.lookupTransform(
                    target_frame_, source_frame_,
                    builtin_interfaces::msg::Time(),
                    rclcpp::Duration::from_seconds(0.05)
                );
                tf2::fromMsg(tf.transform.rotation, rotation_);
                got_tf = true;
                RCLCPP_INFO(this->get_logger(), "Transform acquired!");
                break;
            } catch (tf2::TransformException &ex) {
                rclcpp::sleep_for(std::chrono::milliseconds(50));
            }
        }
        if (!got_tf) {
            RCLCPP_ERROR(this->get_logger(), "Could not find the static transform %s -> %s!", source_frame_.c_str(), target_frame_.c_str());
            throw std::runtime_error("No static transform found at startup.");
        }

        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "imu/data_ned", 10,
            std::bind(&ImuTfTransformNode::imu_callback, this, std::placeholders::_1));
        imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("imu/data_enu", 10);
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    tf2::Quaternion rotation_;
    std::string source_frame_, target_frame_;

    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        auto enu_msg = *msg;
        enu_msg.header.frame_id = target_frame_;

        // --- Orientation ---
        tf2::Quaternion q_imu;
        tf2::fromMsg(msg->orientation, q_imu);
        tf2::Quaternion q_enu = rotation_ * q_imu;
        enu_msg.orientation = tf2::toMsg(q_enu);

        // --- Angular velocity ---
        tf2::Vector3 av_imu(
            msg->angular_velocity.x,
            msg->angular_velocity.y,
            msg->angular_velocity.z);
        tf2::Vector3 av_enu = tf2::quatRotate(rotation_, av_imu);
        enu_msg.angular_velocity.x = av_enu.x();
        enu_msg.angular_velocity.y = av_enu.y();
        enu_msg.angular_velocity.z = av_enu.z();

        // --- Linear acceleration ---
        tf2::Vector3 la_imu(
            msg->linear_acceleration.x,
            msg->linear_acceleration.y,
            msg->linear_acceleration.z);
        tf2::Vector3 la_enu = tf2::quatRotate(rotation_, la_imu);
        enu_msg.linear_acceleration.x = la_enu.x();
        enu_msg.linear_acceleration.y = la_enu.y();
        enu_msg.linear_acceleration.z = la_enu.z();

        imu_pub_->publish(enu_msg);
    }
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    try {
        auto node = std::make_shared<ImuTfTransformNode>();
        rclcpp::spin(node);
    } catch (const std::exception &e) {
        // If transform wasn't available, log and exit cleanly
        RCLCPP_FATAL(rclcpp::get_logger("rclcpp"), "Fatal: %s", e.what());
    }
    rclcpp::shutdown();
    return 0;
}
