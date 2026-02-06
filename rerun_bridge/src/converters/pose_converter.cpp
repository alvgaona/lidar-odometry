#include "rerun_bridge/converters/pose_converter.hpp"

namespace rerun_bridge {

rerun::Transform3D PoseConverter::to_transform(
    const geometry_msgs::msg::PoseStamped::ConstSharedPtr& msg) {

    const auto& pos = msg->pose.position;
    const auto& ori = msg->pose.orientation;

    return rerun::Transform3D::from_translation_rotation(
        rerun::Vec3D(static_cast<float>(pos.x), static_cast<float>(pos.y), static_cast<float>(pos.z)),
        rerun::Quaternion::from_xyzw(
            static_cast<float>(ori.x), static_cast<float>(ori.y),
            static_cast<float>(ori.z), static_cast<float>(ori.w))
    );
}

rerun::Position3D PoseConverter::to_position(
    const geometry_msgs::msg::PoseStamped::ConstSharedPtr& msg) {

    const auto& pos = msg->pose.position;
    return rerun::Position3D(
        static_cast<float>(pos.x), static_cast<float>(pos.y), static_cast<float>(pos.z));
}

}  // namespace rerun_bridge
