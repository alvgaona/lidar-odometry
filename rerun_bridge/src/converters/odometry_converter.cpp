#include "rerun_bridge/converters/odometry_converter.hpp"

namespace rerun_bridge {

rerun::Transform3D OdometryConverter::to_transform(
    const nav_msgs::msg::Odometry::ConstSharedPtr& msg) {

    const auto& pos = msg->pose.pose.position;
    const auto& ori = msg->pose.pose.orientation;

    return rerun::Transform3D::from_translation_rotation(
        rerun::Vec3D(static_cast<float>(pos.x), static_cast<float>(pos.y), static_cast<float>(pos.z)),
        rerun::Quaternion::from_xyzw(
            static_cast<float>(ori.x), static_cast<float>(ori.y),
            static_cast<float>(ori.z), static_cast<float>(ori.w))
    );
}

rerun::Position3D OdometryConverter::to_position(
    const nav_msgs::msg::Odometry::ConstSharedPtr& msg) {

    const auto& pos = msg->pose.pose.position;
    return rerun::Position3D(
        static_cast<float>(pos.x), static_cast<float>(pos.y), static_cast<float>(pos.z));
}

rerun::LineStrips3D OdometryConverter::trajectory_to_line_strips(
    const std::vector<rerun::Position3D>& positions,
    const rerun::Color& color) {

    if (positions.size() < 2) {
        return rerun::LineStrips3D(std::vector<rerun::components::LineStrip3D>{});
    }

    std::vector<rerun::Vec3D> points;
    points.reserve(positions.size());

    for (const auto& pos : positions) {
        points.emplace_back(pos.x(), pos.y(), pos.z());
    }

    std::vector<rerun::components::LineStrip3D> strips;
    strips.emplace_back(points);

    return rerun::LineStrips3D(strips).with_colors({color});
}

}  // namespace rerun_bridge
