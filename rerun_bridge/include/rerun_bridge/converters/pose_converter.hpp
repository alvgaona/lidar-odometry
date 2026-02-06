#pragma once

#include <rerun.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

namespace rerun_bridge {

class PoseConverter {
public:
    static rerun::Transform3D to_transform(
        const geometry_msgs::msg::PoseStamped::ConstSharedPtr& msg);
    static rerun::Position3D to_position(
        const geometry_msgs::msg::PoseStamped::ConstSharedPtr& msg);
};

}  // namespace rerun_bridge
