#include "rerun_bridge/converters/path_converter.hpp"

namespace rerun_bridge {

rerun::LineStrips3D PathConverter::convert(
    const nav_msgs::msg::Path::ConstSharedPtr& msg,
    const rerun::Color& color) {

    if (msg->poses.empty()) {
        return rerun::LineStrips3D(std::vector<rerun::components::LineStrip3D>{});
    }

    std::vector<rerun::Vec3D> points;
    points.reserve(msg->poses.size());

    for (const auto& pose_stamped : msg->poses) {
        const auto& pos = pose_stamped.pose.position;
        points.emplace_back(
            static_cast<float>(pos.x), static_cast<float>(pos.y), static_cast<float>(pos.z));
    }

    std::vector<rerun::components::LineStrip3D> strips;
    strips.emplace_back(points);

    return rerun::LineStrips3D(strips)
        .with_colors({color})
        .with_radii({0.02f});
}

}  // namespace rerun_bridge
