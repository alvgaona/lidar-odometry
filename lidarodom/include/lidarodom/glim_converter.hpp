#pragma once

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <mocap4r2_msgs/msg/rigid_bodies.hpp>
#include <paramify/params.hpp>
#include <memory>


class GlimConverter final : public rclcpp::Node {
public:
    explicit GlimConverter(paramify::Params params);
    ~GlimConverter() override = default;

    static void define_params(paramify::Params& params) {
        params.bind<std::string>("pose_topic")
            .value("/glim_rosbag/pose")
            .help("Topic for GLIM pose input");

        params.bind<std::string>("mocap_topic")
            .value("/mocap/rigid_bodies")
            .help("Topic for mocap rigid bodies input");

        params.bind<std::string>("odom_frame")
            .value("odom")
            .help("Frame ID for odometry path");

        params.bind<int64_t>("rigid_body_index")
            .value(1)
            .help("Index of the rigid body to track")
            .range(int64_t{0}, int64_t{100});
    }

private:
    paramify::Params params_;

    paramify::ParamHandle<std::string> pose_topic_;
    paramify::ParamHandle<std::string> mocap_topic_;
    paramify::ParamHandle<std::string> odom_frame_;
    paramify::ParamHandle<int64_t> rigid_body_index_;

    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_subscriber_;
    rclcpp::Subscription<mocap4r2_msgs::msg::RigidBodies>::SharedPtr mocap_subscriber_;

    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr ground_truth_pose_publisher_;

    std::unique_ptr<nav_msgs::msg::Path> path_msg_;

    void on_pose(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
    void on_mocap(const mocap4r2_msgs::msg::RigidBodies::SharedPtr msg);
};
