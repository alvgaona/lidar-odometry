#include "lidarodom/message_converter.hpp"

MessageConverter::MessageConverter()
    : Node("message_converter"),
      path_msg_(std::make_unique<nav_msgs::msg::Path>()) {

    odom_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/kiss/odometry",
        10,
        [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
            this->publish_pose_and_path(msg);            
        }
    );

    pose_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
        "/kiss/pose",
        10
    );

    path_publisher_ = this->create_publisher<nav_msgs::msg::Path>(
        "/kiss/path",
        10
    );

    // Initialize path frame_id
    path_msg_->header.frame_id = "odom_lidar";

    RCLCPP_INFO(this->get_logger(), "Odometry to Pose/Path converter started");
    RCLCPP_INFO(this->get_logger(), "Subscribing to: /kiss/odometry");
    RCLCPP_INFO(this->get_logger(), "Publishing to: /kiss/pose and /kiss/path");
}

void MessageConverter::publish_pose_and_path(const nav_msgs::msg::Odometry::SharedPtr msg) {
    auto pose_stamped = geometry_msgs::msg::PoseStamped();
    pose_stamped.header = msg->header;
    pose_stamped.pose = msg->pose.pose;
    pose_publisher_->publish(pose_stamped);

    // Add to path and publish
    path_msg_->poses.push_back(pose_stamped);
    path_msg_->header.stamp = msg->header.stamp;
    path_msg_->header.frame_id = msg->header.frame_id;
    path_publisher_->publish(*path_msg_);
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MessageConverter>();

    try {
        rclcpp::spin(node);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(node->get_logger(), "Exception: %s", e.what());
    }

    rclcpp::shutdown();
    return 0;
}
