#pragma once

#include <rclcpp/rclcpp.hpp>
#include <rerun.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <memory>
#include <string>
#include <vector>
#include <unordered_map>

namespace rerun_bridge {

struct TopicConfig {
    std::string topic_name;
    std::string entity_path;
    std::string msg_type;
};

class RerunBridgeNode : public rclcpp::Node {
public:
    explicit RerunBridgeNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
    ~RerunBridgeNode() override = default;

private:
    void initialize_parameters();
    void setup_rerun_stream();
    void create_subscriptions();
    void set_rerun_time(const builtin_interfaces::msg::Time& stamp);

    void on_pointcloud(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& msg,
                       const std::string& entity_path);
    void on_odometry(const nav_msgs::msg::Odometry::ConstSharedPtr& msg,
                     const std::string& entity_path);
    void on_path(const nav_msgs::msg::Path::ConstSharedPtr& msg,
                 const std::string& entity_path);
    void on_imu(const sensor_msgs::msg::Imu::ConstSharedPtr& msg,
                const std::string& entity_path);
    void on_pose(const geometry_msgs::msg::PoseStamped::ConstSharedPtr& msg,
                 const std::string& entity_path);

    void setup_tf_listener();
    void on_tf_timer();

    std::unique_ptr<rerun::RecordingStream> rec_;

    std::string recording_name_;
    std::string connection_mode_;
    std::string connect_address_;
    std::string entity_prefix_;
    std::vector<TopicConfig> topic_configs_;

    std::unordered_map<std::string, std::vector<rerun::Position3D>> trajectories_;

    std::vector<rclcpp::SubscriptionBase::SharedPtr> subscriptions_;

    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
    rclcpp::TimerBase::SharedPtr tf_timer_;
    std::vector<std::pair<std::string, std::string>> tf_frames_;
};

}  // namespace rerun_bridge
