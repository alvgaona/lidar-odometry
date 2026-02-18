#include "lidarodom/kiss_converter.hpp"
#include <paramify/run.hpp>
#include <mocap4r2_msgs/msg/rigid_bodies.hpp>
#include <tf2/LinearMath/Quaternion.h>

KissConverter::KissConverter(paramify::Params params)
    : Node("kiss_converter"),
      params_(std::move(params)),
      path_msg_(std::make_unique<nav_msgs::msg::Path>()),
      tf_broadcaster_(std::make_unique<tf2_ros::TransformBroadcaster>(*this)) {

    params_.parse(this);

    rigid_body_index_ = params_.param<int64_t>("rigid_body_index");
    odom_topic_ = params_.param<std::string>("odom_topic");
    mocap_topic_ = params_.param<std::string>("mocap_topic");
    odom_frame_ = params_.param<std::string>("odom_frame");
    map_frame_ = params_.param<std::string>("map_frame");
    tf_rate_ = params_.param<double>("tf_rate");

    odom_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
        odom_topic_.get(),
        10,
        [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
            this->publish_pose_and_path(msg);
        }
    );

    rigid_bodies_subscriber_ = this->create_subscription<mocap4r2_msgs::msg::RigidBodies>(
        mocap_topic_.get(),
        10,
        [this](const mocap4r2_msgs::msg::RigidBodies::SharedPtr msg) {
            this->publish_rigid_bodies_pose(msg);
        }
    );

    RCLCPP_INFO(this->get_logger(), "Subscribers:");
    RCLCPP_INFO(this->get_logger(), "  - %s", odom_topic_.get().c_str());
    RCLCPP_INFO(this->get_logger(), "  - %s", mocap_topic_.get().c_str());

    pose_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
        "/lidarodom/pose",
        10
    );

    path_publisher_ = this->create_publisher<nav_msgs::msg::Path>(
        "/lidarodom/path",
        10
    );

    ground_truth_pose_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
        "/lidarodom/ground_truth/pose",
        10
    );

    RCLCPP_INFO(this->get_logger(), "Publishers:");
    RCLCPP_INFO(this->get_logger(), "  - /lidarodom/pose");
    RCLCPP_INFO(this->get_logger(), "  - /lidarodom/path");
    RCLCPP_INFO(this->get_logger(), "  - /lidarodom/ground_truth/pose");

    path_msg_->header.frame_id = odom_frame_.get();

    map_to_odom_transform_.header.frame_id = map_frame_.get();
    map_to_odom_transform_.child_frame_id = odom_frame_.get();
    map_to_odom_transform_.transform.translation.x = 0.0;
    map_to_odom_transform_.transform.translation.y = 0.0;
    map_to_odom_transform_.transform.translation.z = 0.0;
    map_to_odom_transform_.transform.rotation.x = 0.0;
    map_to_odom_transform_.transform.rotation.y = 0.0;
    map_to_odom_transform_.transform.rotation.z = 0.0;
    map_to_odom_transform_.transform.rotation.w = 1.0;

    int timer_period_ms = static_cast<int>(1000.0 / tf_rate_.get());
    tf_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(timer_period_ms),
        [this]() {
            map_to_odom_transform_.header.stamp = this->now();
            tf_broadcaster_->sendTransform(map_to_odom_transform_);
        }
    );
}

void KissConverter::publish_rigid_bodies_pose(const mocap4r2_msgs::msg::RigidBodies::SharedPtr msg) {
    auto index = rigid_body_index_.get();
    if (index >= static_cast<int64_t>(msg->rigidbodies.size())) {
        RCLCPP_WARN_ONCE(this->get_logger(), "Rigid body index %lld out of range (size: %zu)",
                         static_cast<long long>(index), msg->rigidbodies.size());
        return;
    }
    const auto& raw_pose = msg->rigidbodies.at(static_cast<size_t>(index)).pose;

    geometry_msgs::msg::PoseStamped pose_stamped;
    pose_stamped.header.stamp = this->now();
    pose_stamped.header.frame_id = odom_frame_.get();
    pose_stamped.pose = raw_pose;

    ground_truth_pose_publisher_->publish(pose_stamped);
}

void KissConverter::publish_map_to_odom_transform(const builtin_interfaces::msg::Time& stamp) {
    map_to_odom_transform_.header.stamp = stamp;
    tf_broadcaster_->sendTransform(map_to_odom_transform_);
}

void KissConverter::publish_pose_and_path(const nav_msgs::msg::Odometry::SharedPtr msg) {
    auto pose_stamped = geometry_msgs::msg::PoseStamped();
    pose_stamped.header.stamp = this->now();
    pose_stamped.header.frame_id = msg->header.frame_id;
    pose_stamped.pose = msg->pose.pose;
    pose_publisher_->publish(pose_stamped);

    path_msg_->poses.push_back(pose_stamped);
    path_msg_->header.stamp = this->now();
    path_msg_->header.frame_id = msg->header.frame_id;
    path_publisher_->publish(*path_msg_);
}

int main(int argc, char** argv) {
    return paramify::run<KissConverter>(
        argc, argv,
        "Convert mocap and odometry messages",
        "lidarodom", "kiss_converter");
}
