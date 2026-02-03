#include "lidarodom/kiss_converter.hpp"
#include <geometry_msgs/msg/detail/pose_stamped__struct.hpp>
#include <mocap4r2_msgs/mocap4r2_msgs/msg/rigid_bodies.hpp>
#include <mocap4r2_msgs/msg/detail/rigid_bodies__struct.hpp>
#include <nav_msgs/msg/detail/path__struct.hpp>
#include <tf2/LinearMath/Quaternion.h>

KissConverter::KissConverter(lidarodom::Params params)
    : Node("kiss_converter"),
      params_(std::move(params)),
      path_msg_(std::make_unique<nav_msgs::msg::Path>()),
      ground_truth_path_msg_(std::make_unique<nav_msgs::msg::Path>()),
      tf_broadcaster_(std::make_unique<tf2_ros::TransformBroadcaster>(*this)) {

    params_.parse(this);

    auto odom_topic = params_.get<std::string>("odom_topic");
    auto mocap_topic = params_.get<std::string>("mocap_topic");
    auto odom_frame = params_.get<std::string>("odom_frame");
    auto map_frame = params_.get<std::string>("map_frame");
    auto tf_rate = params_.get<double>("tf_rate");

    // Subscribers
    odom_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
        odom_topic,
        10,
        [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
            this->publish_pose_and_path(msg);
        }
    );

    rigid_bodies_subscriber_ = this->create_subscription<mocap4r2_msgs::msg::RigidBodies>(
        mocap_topic,
        10,
        [this](const mocap4r2_msgs::msg::RigidBodies::SharedPtr msg) {
            auto pose = this->publish_rigid_bodies_pose(msg);
            if (alignment_initialized_) {
                this->publish_rigid_bodies_path(pose);
            }
        }
    );

    RCLCPP_INFO(this->get_logger(), "Subscribers:");
    RCLCPP_INFO(this->get_logger(), "  - %s", odom_topic.c_str());
    RCLCPP_INFO(this->get_logger(), "  - %s", mocap_topic.c_str());

    // Publishers
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

    ground_truth_path_publisher_ = this->create_publisher<nav_msgs::msg::Path>(
        "/lidarodom/ground_truth/path",
        10
    );

    RCLCPP_INFO(this->get_logger(), "Publishers:");
    RCLCPP_INFO(this->get_logger(), "  - /lidarodom/pose");
    RCLCPP_INFO(this->get_logger(), "  - /lidarodom/path");
    RCLCPP_INFO(this->get_logger(), "  - /lidarodom/ground_truth/pose");
    RCLCPP_INFO(this->get_logger(), "  - /lidarodom/ground_truth/path");

    // Initialize path frame_id (both use odom frame)
    path_msg_->header.frame_id = odom_frame;
    ground_truth_path_msg_->header.frame_id = odom_frame;

    map_to_odom_transform_.header.frame_id = map_frame;
    map_to_odom_transform_.child_frame_id = odom_frame;
    map_to_odom_transform_.transform.translation.x = 0.0;
    map_to_odom_transform_.transform.translation.y = 0.0;
    map_to_odom_transform_.transform.translation.z = 0.0;
    map_to_odom_transform_.transform.rotation.x = 0.0;
    map_to_odom_transform_.transform.rotation.y = 0.0;
    map_to_odom_transform_.transform.rotation.z = 0.0;
    map_to_odom_transform_.transform.rotation.w = 1.0;

    int timer_period_ms = static_cast<int>(1000.0 / tf_rate);
    tf_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(timer_period_ms),
        [this]() {
            map_to_odom_transform_.header.stamp = this->now();
            tf_broadcaster_->sendTransform(map_to_odom_transform_);
        }
    );
}

void KissConverter::publish_rigid_bodies_path(const geometry_msgs::msg::PoseStamped pose) {
    ground_truth_path_msg_->poses.push_back(pose);
    ground_truth_path_msg_->header = pose.header;
    ground_truth_path_publisher_->publish(*ground_truth_path_msg_);
}

geometry_msgs::msg::Pose KissConverter::transform_to_odom_frame(const geometry_msgs::msg::Pose& mocap_pose) {
    tf2::Transform mocap_tf;
    mocap_tf.setOrigin(tf2::Vector3(
        mocap_pose.position.x, mocap_pose.position.y, mocap_pose.position.z));
    mocap_tf.setRotation(tf2::Quaternion(
        mocap_pose.orientation.x, mocap_pose.orientation.y,
        mocap_pose.orientation.z, mocap_pose.orientation.w));

    // Apply alignment: aligned_pose = align_transform * mocap_pose
    tf2::Transform aligned_tf = align_transform_ * mocap_tf;

    geometry_msgs::msg::Pose aligned_pose;
    aligned_pose.position.x = aligned_tf.getOrigin().x();
    aligned_pose.position.y = aligned_tf.getOrigin().y();
    aligned_pose.position.z = aligned_tf.getOrigin().z();
    aligned_pose.orientation.x = aligned_tf.getRotation().x();
    aligned_pose.orientation.y = aligned_tf.getRotation().y();
    aligned_pose.orientation.z = aligned_tf.getRotation().z();
    aligned_pose.orientation.w = aligned_tf.getRotation().w();

    return aligned_pose;
}

geometry_msgs::msg::PoseStamped KissConverter::publish_rigid_bodies_pose(const mocap4r2_msgs::msg::RigidBodies::SharedPtr msg) {
    auto pose_stamped = geometry_msgs::msg::PoseStamped();
    const auto& raw_pose = msg->rigidbodies.at(params_.get<int>("rigid_body_index")).pose;

    // Store latest mocap pose for alignment computation
    latest_mocap_pose_ = raw_pose;
    mocap_received_ = true;

    // Don't publish until alignment is initialized (waiting for first odom)
    if (!alignment_initialized_) {
        return pose_stamped;
    }

    pose_stamped.header.stamp = this->now();
    pose_stamped.header.frame_id = params_.get<std::string>("odom_frame");

    // Transform mocap pose to odom_lidar frame
    pose_stamped.pose = transform_to_odom_frame(raw_pose);

    ground_truth_pose_publisher_->publish(pose_stamped);

    return pose_stamped;
}

void KissConverter::publish_map_to_odom_transform(const builtin_interfaces::msg::Time& stamp) {
    map_to_odom_transform_.header.stamp = stamp;
    tf_broadcaster_->sendTransform(map_to_odom_transform_);
}

void KissConverter::publish_pose_and_path(const nav_msgs::msg::Odometry::SharedPtr msg) {
    // Initialize alignment on first odometry message
    if (!alignment_initialized_ && mocap_received_) {
        // Get odometry pose
        tf2::Transform odom_tf;
        odom_tf.setOrigin(tf2::Vector3(
            msg->pose.pose.position.x,
            msg->pose.pose.position.y,
            msg->pose.pose.position.z));
        odom_tf.setRotation(tf2::Quaternion(
            msg->pose.pose.orientation.x,
            msg->pose.pose.orientation.y,
            msg->pose.pose.orientation.z,
            msg->pose.pose.orientation.w));

        // Get mocap pose at this moment
        tf2::Transform mocap_tf;
        mocap_tf.setOrigin(tf2::Vector3(
            latest_mocap_pose_.position.x,
            latest_mocap_pose_.position.y,
            latest_mocap_pose_.position.z));
        mocap_tf.setRotation(tf2::Quaternion(
            latest_mocap_pose_.orientation.x,
            latest_mocap_pose_.orientation.y,
            latest_mocap_pose_.orientation.z,
            latest_mocap_pose_.orientation.w));

        // Compute alignment: align_transform = odom_pose * mocap_pose.inverse()
        align_transform_ = odom_tf * mocap_tf.inverse();

        // Update TF transform (map -> odom)
        map_to_odom_transform_.transform.translation.x = latest_mocap_pose_.position.x;
        map_to_odom_transform_.transform.translation.y = latest_mocap_pose_.position.y;
        map_to_odom_transform_.transform.translation.z = latest_mocap_pose_.position.z;
        map_to_odom_transform_.transform.rotation = latest_mocap_pose_.orientation;

        alignment_initialized_ = true;
        RCLCPP_INFO(this->get_logger(), "Alignment initialized: mocap -> odom_lidar");
    }

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
    lidarodom::Params params;

    params.bind<int>("rigid_body_index")
        .value(1)
        .help("Index of the rigid body to track")
        .range(0, 100);

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

    if (lidarodom::Params::has_help_flag(argc, argv)) {
        params.print_help("Convert mocap and odometry messages", "lidarodom", "kiss_converter");
        return 0;
    }

    rclcpp::init(argc, argv);
    auto node = std::make_shared<KissConverter>(std::move(params));

    try {
        rclcpp::spin(node);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(node->get_logger(), "Exception: %s", e.what());
    }

    rclcpp::shutdown();
    return 0;
}
