#pragma once

#include <rerun.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <optional>
#include <vector>

namespace rerun_bridge {

class PointCloudConverter {
public:
    static rerun::Points3D convert(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& msg);
    static std::vector<rerun::Position3D> extract_positions(
        const sensor_msgs::msg::PointCloud2::ConstSharedPtr& msg);
    static std::optional<std::vector<rerun::Color>> extract_colors(
        const sensor_msgs::msg::PointCloud2::ConstSharedPtr& msg);
};

}  // namespace rerun_bridge
