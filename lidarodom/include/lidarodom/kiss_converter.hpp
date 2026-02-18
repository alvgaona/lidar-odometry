#pragma once

#include <mocap4r2_msgs/msg/rigid_bodies.hpp>
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <builtin_interfaces/msg/time.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Transform.h>
#include <paramify/params.hpp>
#include <memory>


class KissConverter final : public rclcpp::Node {
public:
    explicit KissConverter(paramify::Params params);
    ~KissConverter() override = default;

    static void define_params(paramify::Params& params) {
        params.bind<int64_t>("rigid_body_index")
            .value(1)
            .help("Index of the rigid body to track")
            .range(int64_t{0}, int64_t{100});

        params.bind<std::string>("odom_topic")
            .value("/kiss/odometry")
            .help("Topic for odometry input");

        params.bind<std::string>("mocap_topic")
            .value("/mocap/rigid_bodies")
            .help("Topic for mocap rigid bodies input");

        params.bind<std::string>("odom_frame")
            .value("odom_lidar")
            .help("Frame ID for odometry");

        params.bind<std::string>("map_frame")
            .value("map")
            .help("Frame ID for map");

        params.bind<double>("tf_rate")
            .value(50.0)
            .help("TF broadcast rate in Hz")
            .range(1.0, 1000.0);
    }

private:
    paramify::Params params_;

    paramify::ParamHandle<int64_t> rigid_body_index_;
    paramify::ParamHandle<std::string> odom_topic_;
    paramify::ParamHandle<std::string> mocap_topic_;
    paramify::ParamHandle<std::string> odom_frame_;
    paramify::ParamHandle<std::string> map_frame_;
    paramify::ParamHandle<double> tf_rate_;

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscriber_;
    rclcpp::Subscription<mocap4r2_msgs::msg::RigidBodies>::SharedPtr rigid_bodies_subscriber_;

    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_publisher_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr ground_truth_pose_publisher_;

    std::unique_ptr<nav_msgs::msg::Path> path_msg_;

    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    geometry_msgs::msg::TransformStamped map_to_odom_transform_;

    rclcpp::TimerBase::SharedPtr tf_timer_;

    void publish_pose_and_path(const nav_msgs::msg::Odometry::SharedPtr msg);
    void publish_rigid_bodies_pose(const mocap4r2_msgs::msg::RigidBodies::SharedPtr msg);
    void publish_map_to_odom_transform(const builtin_interfaces::msg::Time& stamp);
};
