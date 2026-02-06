#pragma once

#include <rerun.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include <string>

namespace rerun_bridge {

class ImuConverter {
public:
    static void log_to_rerun(
        rerun::RecordingStream& rec,
        const sensor_msgs::msg::Imu::ConstSharedPtr& msg,
        const std::string& entity_path);
    static rerun::Transform3D orientation_to_transform(
        const sensor_msgs::msg::Imu::ConstSharedPtr& msg);
};

}  // namespace rerun_bridge
