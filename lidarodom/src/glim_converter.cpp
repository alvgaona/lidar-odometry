#include "lidarodom/glim_converter.hpp"

GlimConverter::GlimConverter()
    : Node("glim_converter", rclcpp::NodeOptions().parameter_overrides({{"use_sim_time", true}})),
      path_msg_(std::make_unique<nav_msgs::msg::Path>()) {

    pose_topic_ = this->declare_parameter<std::string>("pose_topic", "/glim_rosbag/pose");
    mocap_topic_ = this->declare_parameter<std::string>("mocap_topic", "/mocap/rigid_bodies");
    odom_frame_ = this->declare_parameter<std::string>("odom_frame", "odom");
    rigid_body_index_ = this->declare_parameter<int>("rigid_body_index", 1);

    pose_subscriber_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        pose_topic_, 10,
        [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) { on_pose(msg); }
    );

    mocap_subscriber_ = this->create_subscription<mocap4r2_msgs::msg::RigidBodies>(
        mocap_topic_, 10,
        [this](const mocap4r2_msgs::msg::RigidBodies::SharedPtr msg) { on_mocap(msg); }
    );

    RCLCPP_INFO(this->get_logger(), "Subscribers:");
    RCLCPP_INFO(this->get_logger(), "  - %s (pose)", pose_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "  - %s (mocap)", mocap_topic_.c_str());

    path_publisher_ = this->create_publisher<nav_msgs::msg::Path>("/lidarodom/path", 10);
    ground_truth_pose_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
        "/lidarodom/ground_truth/pose", 10);

    RCLCPP_INFO(this->get_logger(), "Publishers:");
    RCLCPP_INFO(this->get_logger(), "  - /lidarodom/path");
    RCLCPP_INFO(this->get_logger(), "  - /lidarodom/ground_truth/pose");

    path_msg_->header.frame_id = odom_frame_;

    RCLCPP_INFO(this->get_logger(), "Odom frame: %s", odom_frame_.c_str());
}

void GlimConverter::on_pose(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    auto now = this->now();
    auto pose = *msg;
    pose.header.stamp = now;
    pose.header.frame_id = odom_frame_;
    path_msg_->poses.push_back(pose);
    path_msg_->header.stamp = now;
    path_publisher_->publish(*path_msg_);
}

void GlimConverter::on_mocap(const mocap4r2_msgs::msg::RigidBodies::SharedPtr msg) {
    if (rigid_body_index_ >= static_cast<int>(msg->rigidbodies.size())) {
        RCLCPP_WARN_ONCE(this->get_logger(), "Rigid body index %d out of range", rigid_body_index_);
        return;
    }

    geometry_msgs::msg::PoseStamped pose_stamped;
    pose_stamped.header.stamp = this->now();
    pose_stamped.header.frame_id = msg->header.frame_id;
    pose_stamped.pose = msg->rigidbodies.at(rigid_body_index_).pose;

    ground_truth_pose_publisher_->publish(pose_stamped);
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<GlimConverter>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
