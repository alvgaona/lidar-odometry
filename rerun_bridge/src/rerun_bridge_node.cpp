#include "rerun_bridge/rerun_bridge_node.hpp"
#include "rerun_bridge/converters/pointcloud_converter.hpp"
#include "rerun_bridge/converters/odometry_converter.hpp"
#include "rerun_bridge/converters/path_converter.hpp"
#include "rerun_bridge/converters/imu_converter.hpp"
#include "rerun_bridge/converters/pose_converter.hpp"

#include <chrono>

namespace rerun_bridge {

RerunBridgeNode::RerunBridgeNode(const rclcpp::NodeOptions& options)
    : Node("rerun_bridge", options) {

    initialize_parameters();
    setup_rerun_stream();
    setup_tf_listener();
    create_subscriptions();

    RCLCPP_INFO(get_logger(), "Rerun bridge node initialized");
}

void RerunBridgeNode::initialize_parameters() {
    recording_name_ = declare_parameter<std::string>("recording_name", "lidar_odometry");
    connection_mode_ = declare_parameter<std::string>("connection_mode", "spawn");
    connect_address_ = declare_parameter<std::string>(
        "connect_address", "rerun+http://127.0.0.1:9876/proxy");
    entity_prefix_ = declare_parameter<std::string>("entity_prefix", "/world");

    auto pointcloud_topics = declare_parameter<std::vector<std::string>>(
        "pointcloud_topics", std::vector<std::string>{"/livox/lidar"});
    auto pointcloud_entities = declare_parameter<std::vector<std::string>>(
        "pointcloud_entities", std::vector<std::string>{"/lidar/points"});

    auto odometry_topics = declare_parameter<std::vector<std::string>>(
        "odometry_topics", std::vector<std::string>{"/kiss/odometry"});
    auto odometry_entities = declare_parameter<std::vector<std::string>>(
        "odometry_entities", std::vector<std::string>{"/odom/pose"});

    auto path_topics = declare_parameter<std::vector<std::string>>(
        "path_topics", std::vector<std::string>{"/lidarodom/path"});
    auto path_entities = declare_parameter<std::vector<std::string>>(
        "path_entities", std::vector<std::string>{"/odom/trajectory"});

    auto imu_topics = declare_parameter<std::vector<std::string>>(
        "imu_topics", std::vector<std::string>{"/livox/imu"});
    auto imu_entities = declare_parameter<std::vector<std::string>>(
        "imu_entities", std::vector<std::string>{"/imu"});

    auto pose_topics = declare_parameter<std::vector<std::string>>(
        "pose_topics", std::vector<std::string>{});
    auto pose_entities = declare_parameter<std::vector<std::string>>(
        "pose_entities", std::vector<std::string>{});

    for (size_t i = 0; i < pointcloud_topics.size(); ++i) {
        topic_configs_.push_back({
            pointcloud_topics[i],
            i < pointcloud_entities.size() ? pointcloud_entities[i] : "/lidar/points",
            "PointCloud2"
        });
    }

    for (size_t i = 0; i < odometry_topics.size(); ++i) {
        topic_configs_.push_back({
            odometry_topics[i],
            i < odometry_entities.size() ? odometry_entities[i] : "/odom/pose",
            "Odometry"
        });
    }

    for (size_t i = 0; i < path_topics.size(); ++i) {
        topic_configs_.push_back({
            path_topics[i],
            i < path_entities.size() ? path_entities[i] : "/path",
            "Path"
        });
    }

    for (size_t i = 0; i < imu_topics.size(); ++i) {
        topic_configs_.push_back({
            imu_topics[i],
            i < imu_entities.size() ? imu_entities[i] : "/imu",
            "Imu"
        });
    }

    for (size_t i = 0; i < pose_topics.size(); ++i) {
        topic_configs_.push_back({
            pose_topics[i],
            i < pose_entities.size() ? pose_entities[i] : "/pose",
            "PoseStamped"
        });
    }

    RCLCPP_INFO(get_logger(), "Configured %zu topics", topic_configs_.size());
}

void RerunBridgeNode::setup_rerun_stream() {
    rec_ = std::make_unique<rerun::RecordingStream>(recording_name_);

    if (connection_mode_ == "spawn") {
        auto err = rec_->spawn();
        if (err.is_err()) {
            RCLCPP_ERROR(get_logger(), "Failed to spawn Rerun viewer: %s",
                         err.description.c_str());
        } else {
            RCLCPP_INFO(get_logger(), "Spawned Rerun viewer");
        }
    } else if (connection_mode_ == "connect") {
        auto err = rec_->connect_grpc(connect_address_);
        if (err.is_err()) {
            RCLCPP_ERROR(get_logger(), "Failed to connect to Rerun: %s",
                         err.description.c_str());
        } else {
            RCLCPP_INFO(get_logger(), "Connected to Rerun at %s", connect_address_.c_str());
        }
    } else if (connection_mode_ == "save") {
        auto save_path = declare_parameter<std::string>("save_path", "recording.rrd");
        auto err = rec_->save(save_path);
        if (err.is_err()) {
            RCLCPP_ERROR(get_logger(), "Failed to save to %s: %s",
                         save_path.c_str(), err.description.c_str());
        } else {
            RCLCPP_INFO(get_logger(), "Saving recording to %s", save_path.c_str());
        }
    }
}

void RerunBridgeNode::set_rerun_time(const builtin_interfaces::msg::Time& stamp) {
    int64_t nanos = static_cast<int64_t>(stamp.sec) * 1000000000LL + stamp.nanosec;
    rec_->set_time_timestamp_nanos_since_epoch("ros_time", nanos);
}

void RerunBridgeNode::create_subscriptions() {
    for (const auto& config : topic_configs_) {
        std::string full_entity = entity_prefix_ + config.entity_path;

        if (config.msg_type == "PointCloud2") {
            auto sub = create_subscription<sensor_msgs::msg::PointCloud2>(
                config.topic_name,
                rclcpp::SensorDataQoS(),
                [this, full_entity](const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) {
                    on_pointcloud(msg, full_entity);
                }
            );
            subscriptions_.push_back(sub);
            RCLCPP_INFO(get_logger(), "Subscribed to PointCloud2: %s -> %s",
                        config.topic_name.c_str(), full_entity.c_str());
        }
        else if (config.msg_type == "Odometry") {
            auto sub = create_subscription<nav_msgs::msg::Odometry>(
                config.topic_name,
                10,
                [this, full_entity](const nav_msgs::msg::Odometry::ConstSharedPtr msg) {
                    on_odometry(msg, full_entity);
                }
            );
            subscriptions_.push_back(sub);
            RCLCPP_INFO(get_logger(), "Subscribed to Odometry: %s -> %s",
                        config.topic_name.c_str(), full_entity.c_str());
        }
        else if (config.msg_type == "Path") {
            auto sub = create_subscription<nav_msgs::msg::Path>(
                config.topic_name,
                10,
                [this, full_entity](const nav_msgs::msg::Path::ConstSharedPtr msg) {
                    on_path(msg, full_entity);
                }
            );
            subscriptions_.push_back(sub);
            RCLCPP_INFO(get_logger(), "Subscribed to Path: %s -> %s",
                        config.topic_name.c_str(), full_entity.c_str());
        }
        else if (config.msg_type == "Imu") {
            auto sub = create_subscription<sensor_msgs::msg::Imu>(
                config.topic_name,
                rclcpp::SensorDataQoS(),
                [this, full_entity](const sensor_msgs::msg::Imu::ConstSharedPtr msg) {
                    on_imu(msg, full_entity);
                }
            );
            subscriptions_.push_back(sub);
            RCLCPP_INFO(get_logger(), "Subscribed to Imu: %s -> %s",
                        config.topic_name.c_str(), full_entity.c_str());
        }
        else if (config.msg_type == "PoseStamped") {
            auto sub = create_subscription<geometry_msgs::msg::PoseStamped>(
                config.topic_name,
                10,
                [this, full_entity](const geometry_msgs::msg::PoseStamped::ConstSharedPtr msg) {
                    on_pose(msg, full_entity);
                }
            );
            subscriptions_.push_back(sub);
            RCLCPP_INFO(get_logger(), "Subscribed to PoseStamped: %s -> %s",
                        config.topic_name.c_str(), full_entity.c_str());
        }
    }
}

void RerunBridgeNode::on_pointcloud(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr& msg,
    const std::string& entity_path) {

    set_rerun_time(msg->header.stamp);
    auto points = PointCloudConverter::convert(msg);
    rec_->log(entity_path, points);
}

void RerunBridgeNode::on_odometry(
    const nav_msgs::msg::Odometry::ConstSharedPtr& msg,
    const std::string& entity_path) {

    set_rerun_time(msg->header.stamp);

    auto transform = OdometryConverter::to_transform(msg);
    rec_->log(entity_path, transform);

    auto pos = OdometryConverter::to_position(msg);
    trajectories_[entity_path].push_back(pos);

    if (trajectories_[entity_path].size() > 1) {
        auto trajectory = OdometryConverter::trajectory_to_line_strips(
            trajectories_[entity_path], rerun::Color(0, 255, 0));
        rec_->log(entity_path + "/trajectory", trajectory);
    }
}

void RerunBridgeNode::on_path(
    const nav_msgs::msg::Path::ConstSharedPtr& msg,
    const std::string& entity_path) {

    if (!msg->poses.empty()) {
        set_rerun_time(msg->poses.back().header.stamp);
    }

    auto line_strips = PathConverter::convert(msg, rerun::Color(255, 128, 0));
    rec_->log(entity_path, line_strips);
}

void RerunBridgeNode::on_imu(
    const sensor_msgs::msg::Imu::ConstSharedPtr& msg,
    const std::string& entity_path) {

    set_rerun_time(msg->header.stamp);
    ImuConverter::log_to_rerun(*rec_, msg, entity_path);
}

void RerunBridgeNode::on_pose(
    const geometry_msgs::msg::PoseStamped::ConstSharedPtr& msg,
    const std::string& entity_path) {

    set_rerun_time(msg->header.stamp);
    auto transform = PoseConverter::to_transform(msg);
    rec_->log(entity_path, transform);

    auto pos = PoseConverter::to_position(msg);
    trajectories_[entity_path].push_back(pos);

    if (trajectories_[entity_path].size() > 1) {
        auto trajectory = OdometryConverter::trajectory_to_line_strips(
            trajectories_[entity_path], rerun::Color(255, 0, 255));
        rec_->log(entity_path + "/trajectory", trajectory);
    }
}

void RerunBridgeNode::setup_tf_listener() {
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);

    auto tf_parent_frames = declare_parameter<std::vector<std::string>>(
        "tf_parent_frames", std::vector<std::string>{});
    auto tf_child_frames = declare_parameter<std::vector<std::string>>(
        "tf_child_frames", std::vector<std::string>{});

    for (size_t i = 0; i < tf_parent_frames.size() && i < tf_child_frames.size(); ++i) {
        tf_frames_.emplace_back(tf_parent_frames[i], tf_child_frames[i]);
    }

    if (!tf_frames_.empty()) {
        tf_timer_ = create_wall_timer(
            std::chrono::milliseconds(50),
            std::bind(&RerunBridgeNode::on_tf_timer, this)
        );
        RCLCPP_INFO(get_logger(), "Tracking %zu TF frame pairs", tf_frames_.size());
    }
}

void RerunBridgeNode::on_tf_timer() {
    for (const auto& [parent, child] : tf_frames_) {
        try {
            auto tf = tf_buffer_->lookupTransform(parent, child, tf2::TimePointZero);

            int64_t nanos = static_cast<int64_t>(tf.header.stamp.sec) * 1000000000LL +
                            tf.header.stamp.nanosec;
            rec_->set_time_timestamp_nanos_since_epoch("ros_time", nanos);

            const auto& t = tf.transform;
            auto transform = rerun::Transform3D::from_translation_rotation(
                rerun::Vec3D(
                    static_cast<float>(t.translation.x),
                    static_cast<float>(t.translation.y),
                    static_cast<float>(t.translation.z)),
                rerun::Quaternion::from_xyzw(
                    static_cast<float>(t.rotation.x),
                    static_cast<float>(t.rotation.y),
                    static_cast<float>(t.rotation.z),
                    static_cast<float>(t.rotation.w))
            );

            std::string entity = entity_prefix_ + "/tf/" + parent + "_to_" + child;
            rec_->log(entity, transform);

        } catch (const tf2::TransformException& ex) {
            RCLCPP_DEBUG(get_logger(), "TF lookup failed: %s", ex.what());
        }
    }
}

}  // namespace rerun_bridge

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rerun_bridge::RerunBridgeNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
