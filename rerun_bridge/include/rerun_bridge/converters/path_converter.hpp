#pragma once

#include <rerun.hpp>
#include <nav_msgs/msg/path.hpp>

namespace rerun_bridge {

class PathConverter {
public:
    static rerun::LineStrips3D convert(
        const nav_msgs::msg::Path::ConstSharedPtr& msg,
        const rerun::Color& color = rerun::Color(255, 128, 0));
};

}  // namespace rerun_bridge
