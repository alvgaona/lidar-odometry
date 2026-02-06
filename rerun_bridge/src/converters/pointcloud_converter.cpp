#include "rerun_bridge/converters/pointcloud_converter.hpp"

#include <algorithm>
#include <cmath>

namespace rerun_bridge {

rerun::Points3D PointCloudConverter::convert(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr& msg) {

    auto positions = extract_positions(msg);
    auto colors = extract_colors(msg);

    if (colors.has_value()) {
        return rerun::Points3D(positions)
            .with_colors(colors.value())
            .with_radii({0.01f});
    }

    return rerun::Points3D(positions).with_radii({0.01f});
}

std::vector<rerun::Position3D> PointCloudConverter::extract_positions(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr& msg) {

    std::vector<rerun::Position3D> positions;
    positions.reserve(msg->height * msg->width);

    sensor_msgs::PointCloud2ConstIterator<float> iter_x(*msg, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iter_y(*msg, "y");
    sensor_msgs::PointCloud2ConstIterator<float> iter_z(*msg, "z");

    for (size_t i = 0; i < msg->height * msg->width; ++i, ++iter_x, ++iter_y, ++iter_z) {
        if (std::isfinite(*iter_x) && std::isfinite(*iter_y) && std::isfinite(*iter_z)) {
            positions.emplace_back(*iter_x, *iter_y, *iter_z);
        }
    }

    return positions;
}

std::optional<std::vector<rerun::Color>> PointCloudConverter::extract_colors(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr& msg) {

    bool has_intensity = false;
    std::string intensity_field_name;
    for (const auto& field : msg->fields) {
        if (field.name == "intensity" || field.name == "i") {
            has_intensity = true;
            intensity_field_name = field.name;
            break;
        }
    }

    if (!has_intensity) {
        return std::nullopt;
    }

    std::vector<rerun::Color> colors;
    colors.reserve(msg->height * msg->width);

    sensor_msgs::PointCloud2ConstIterator<float> iter_i(*msg, intensity_field_name);

    for (size_t i = 0; i < msg->height * msg->width; ++i, ++iter_i) {
        float intensity = *iter_i;
        uint8_t gray = static_cast<uint8_t>(std::clamp(intensity, 0.0f, 255.0f));
        colors.emplace_back(gray, gray, gray);
    }

    return colors;
}

}  // namespace rerun_bridge
