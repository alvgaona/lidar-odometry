#pragma once

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <mocap4r2_msgs/msg/rigid_bodies.hpp>
#include <memory>


class GlimConverter final : public rclcpp::Node {
public:
    GlimConverter();
    ~GlimConverter() override = default;

private:
    std::string pose_topic_;
    std::string mocap_topic_;
    std::string odom_frame_;
    int rigid_body_index_;

    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_subscriber_;
    rclcpp::Subscription<mocap4r2_msgs::msg::RigidBodies>::SharedPtr mocap_subscriber_;

    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr ground_truth_pose_publisher_;

    std::unique_ptr<nav_msgs::msg::Path> path_msg_;

    void on_pose(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
    void on_mocap(const mocap4r2_msgs::msg::RigidBodies::SharedPtr msg);
};
