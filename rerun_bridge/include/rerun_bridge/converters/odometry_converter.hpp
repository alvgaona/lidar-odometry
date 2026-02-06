#pragma once

#include <rerun.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <vector>

namespace rerun_bridge {

class OdometryConverter {
public:
    static rerun::Transform3D to_transform(const nav_msgs::msg::Odometry::ConstSharedPtr& msg);
    static rerun::Position3D to_position(const nav_msgs::msg::Odometry::ConstSharedPtr& msg);
    static rerun::LineStrips3D trajectory_to_line_strips(
        const std::vector<rerun::Position3D>& positions,
        const rerun::Color& color = rerun::Color(0, 255, 0));
};

}  // namespace rerun_bridge
