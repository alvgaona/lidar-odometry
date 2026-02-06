#include "rerun_bridge/converters/imu_converter.hpp"

namespace rerun_bridge {

void ImuConverter::log_to_rerun(
    rerun::RecordingStream& rec,
    const sensor_msgs::msg::Imu::ConstSharedPtr& msg,
    const std::string& entity_path) {

    const auto& accel = msg->linear_acceleration;
    const auto& gyro = msg->angular_velocity;

    rec.log(entity_path + "/accel/x", rerun::Scalars(accel.x));
    rec.log(entity_path + "/accel/y", rerun::Scalars(accel.y));
    rec.log(entity_path + "/accel/z", rerun::Scalars(accel.z));

    rec.log(entity_path + "/gyro/x", rerun::Scalars(gyro.x));
    rec.log(entity_path + "/gyro/y", rerun::Scalars(gyro.y));
    rec.log(entity_path + "/gyro/z", rerun::Scalars(gyro.z));

    rec.log(entity_path + "/orientation", orientation_to_transform(msg));
}

rerun::Transform3D ImuConverter::orientation_to_transform(
    const sensor_msgs::msg::Imu::ConstSharedPtr& msg) {

    const auto& ori = msg->orientation;
    return rerun::Transform3D::from_rotation(
        rerun::Quaternion::from_xyzw(
            static_cast<float>(ori.x), static_cast<float>(ori.y),
            static_cast<float>(ori.z), static_cast<float>(ori.w))
    );
}

}  // namespace rerun_bridge
