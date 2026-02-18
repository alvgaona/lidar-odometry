#include "lidarodom/kiss_converter.hpp"
#include <mocap4r2_msgs/msg/rigid_bodies.hpp>
#include <tf2/LinearMath/Quaternion.h>

KissConverter::KissConverter()
    : Node("kiss_converter"),
      path_msg_(std::make_unique<nav_msgs::msg::Path>()),
      tf_broadcaster_(std::make_unique<tf2_ros::TransformBroadcaster>(*this)) {

    rigid_body_index_ = this->declare_parameter<int>("rigid_body_index", 1);
    odom_topic_ = this->declare_parameter<std::string>("odom_topic", "/kiss/odometry");
    mocap_topic_ = this->declare_parameter<std::string>("mocap_topic", "/mocap/rigid_bodies");
    odom_frame_ = this->declare_parameter<std::string>("odom_frame", "odom_lidar");
    map_frame_ = this->declare_parameter<std::string>("map_frame", "map");
    tf_rate_ = this->declare_parameter<double>("tf_rate", 50.0);

    odom_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
        odom_topic_,
        10,
        [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
            this->publish_pose_and_path(msg);
        }
    );

    rigid_bodies_subscriber_ = this->create_subscription<mocap4r2_msgs::msg::RigidBodies>(
        mocap_topic_,
        10,
        [this](const mocap4r2_msgs::msg::RigidBodies::SharedPtr msg) {
            this->publish_rigid_bodies_pose(msg);
        }
    );

    RCLCPP_INFO(this->get_logger(), "Subscribers:");
    RCLCPP_INFO(this->get_logger(), "  - %s", odom_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "  - %s", mocap_topic_.c_str());

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

    path_msg_->header.frame_id = odom_frame_;

    map_to_odom_transform_.header.frame_id = map_frame_;
    map_to_odom_transform_.child_frame_id = odom_frame_;
    map_to_odom_transform_.transform.translation.x = 0.0;
    map_to_odom_transform_.transform.translation.y = 0.0;
    map_to_odom_transform_.transform.translation.z = 0.0;
    map_to_odom_transform_.transform.rotation.x = 0.0;
    map_to_odom_transform_.transform.rotation.y = 0.0;
    map_to_odom_transform_.transform.rotation.z = 0.0;
    map_to_odom_transform_.transform.rotation.w = 1.0;

    int timer_period_ms = static_cast<int>(1000.0 / tf_rate_);
    tf_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(timer_period_ms),
        [this]() {
            map_to_odom_transform_.header.stamp = this->now();
            tf_broadcaster_->sendTransform(map_to_odom_transform_);
        }
    );
}

void KissConverter::publish_rigid_bodies_pose(const mocap4r2_msgs::msg::RigidBodies::SharedPtr msg) {
    if (rigid_body_index_ >= static_cast<int>(msg->rigidbodies.size())) {
        RCLCPP_WARN_ONCE(this->get_logger(), "Rigid body index %d out of range (size: %zu)",
                         rigid_body_index_, msg->rigidbodies.size());
        return;
    }
    const auto& raw_pose = msg->rigidbodies.at(rigid_body_index_).pose;

    geometry_msgs::msg::PoseStamped pose_stamped;
    pose_stamped.header.stamp = this->now();
    pose_stamped.header.frame_id = odom_frame_;
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
    rclcpp::init(argc, argv);
    auto node = std::make_shared<KissConverter>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
