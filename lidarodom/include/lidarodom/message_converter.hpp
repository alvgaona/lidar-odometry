#pragma once

#include <geometry_msgs/msg/detail/pose_stamped__struct.hpp>
#include <mocap4r2_msgs/msg/detail/rigid_bodies__struct.hpp>
#include <mocap4r2_msgs/msg/rigid_bodies.hpp>
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <builtin_interfaces/msg/time.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <lidarodom/params.hpp>
#include <memory>


class MessageConverter final : public rclcpp::Node {
public:
    explicit MessageConverter(lidarodom::Params params);
    ~MessageConverter() override = default;

private:
    lidarodom::Params params_;

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscriber_;
    rclcpp::Subscription<mocap4r2_msgs::msg::RigidBodies>::SharedPtr rigid_bodies_subscriber_;

    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_publisher_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_publisher_;

    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr ground_truth_pose_publisher_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr ground_truth_path_publisher_;

    std::unique_ptr<nav_msgs::msg::Path> path_msg_;
    std::unique_ptr<nav_msgs::msg::Path> ground_truth_path_msg_;

    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    geometry_msgs::msg::TransformStamped map_to_odom_transform_;
    bool transform_initialized_ = false;

    rclcpp::TimerBase::SharedPtr tf_timer_;

    void publish_pose_and_path(const nav_msgs::msg::Odometry::SharedPtr msg);

    void publish_rigid_bodies_path(const geometry_msgs::msg::PoseStamped pose);

    geometry_msgs::msg::PoseStamped publish_rigid_bodies_pose(const mocap4r2_msgs::msg::RigidBodies::SharedPtr msg);

    void publish_map_to_odom_transform(const builtin_interfaces::msg::Time& stamp);
};
