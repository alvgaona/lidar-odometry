#pragma once

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <memory>


class MessageConverter final : public rclcpp::Node {
public:
    MessageConverter();
    ~MessageConverter() override = default;

private:
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscriber_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_publisher_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_publisher_;

    void publish_pose_and_path(const nav_msgs::msg::Odometry::SharedPtr msg);

    std::unique_ptr<nav_msgs::msg::Path> path_msg_;
};