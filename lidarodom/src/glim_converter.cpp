#include "lidarodom/glim_converter.hpp"
#include <paramify/run.hpp>

GlimConverter::GlimConverter(paramify::Params params)
    : Node("glim_converter", rclcpp::NodeOptions().parameter_overrides({{"use_sim_time", true}})),
      params_(std::move(params)),
      path_msg_(std::make_unique<nav_msgs::msg::Path>()) {

    params_.parse(this);

    pose_topic_ = params_.param<std::string>("pose_topic");
    mocap_topic_ = params_.param<std::string>("mocap_topic");
    odom_frame_ = params_.param<std::string>("odom_frame");
    rigid_body_index_ = params_.param<int64_t>("rigid_body_index");

    pose_subscriber_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        pose_topic_.get(), 10,
        [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) { on_pose(msg); }
    );

    mocap_subscriber_ = this->create_subscription<mocap4r2_msgs::msg::RigidBodies>(
        mocap_topic_.get(), 10,
        [this](const mocap4r2_msgs::msg::RigidBodies::SharedPtr msg) { on_mocap(msg); }
    );

    RCLCPP_INFO(this->get_logger(), "Subscribers:");
    RCLCPP_INFO(this->get_logger(), "  - %s (pose)", pose_topic_.get().c_str());
    RCLCPP_INFO(this->get_logger(), "  - %s (mocap)", mocap_topic_.get().c_str());

    path_publisher_ = this->create_publisher<nav_msgs::msg::Path>("/lidarodom/path", 10);
    ground_truth_pose_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
        "/lidarodom/ground_truth/pose", 10);

    RCLCPP_INFO(this->get_logger(), "Publishers:");
    RCLCPP_INFO(this->get_logger(), "  - /lidarodom/path");
    RCLCPP_INFO(this->get_logger(), "  - /lidarodom/ground_truth/pose");

    path_msg_->header.frame_id = odom_frame_.get();

    RCLCPP_INFO(this->get_logger(), "Odom frame: %s", odom_frame_.get().c_str());
}

void GlimConverter::on_pose(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    auto now = this->now();
    auto pose = *msg;
    pose.header.stamp = now;
    pose.header.frame_id = odom_frame_.get();
    path_msg_->poses.push_back(pose);
    path_msg_->header.stamp = now;
    path_publisher_->publish(*path_msg_);
}

void GlimConverter::on_mocap(const mocap4r2_msgs::msg::RigidBodies::SharedPtr msg) {
    auto index = rigid_body_index_.get();
    if (index >= static_cast<int64_t>(msg->rigidbodies.size())) {
        RCLCPP_WARN_ONCE(this->get_logger(), "Rigid body index %lld out of range",
                         static_cast<long long>(index));
        return;
    }

    geometry_msgs::msg::PoseStamped pose_stamped;
    pose_stamped.header.stamp = this->now();
    pose_stamped.header.frame_id = msg->header.frame_id;
    pose_stamped.pose = msg->rigidbodies.at(static_cast<size_t>(index)).pose;

    ground_truth_pose_publisher_->publish(pose_stamped);
}

int main(int argc, char** argv) {
    return paramify::run<GlimConverter>(
        argc, argv,
        "Convert GLIM pose to path and republish mocap pose",
        "lidarodom", "glim_converter");
}
