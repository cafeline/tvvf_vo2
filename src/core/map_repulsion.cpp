#include "tvvf_vo_c/core/map_repulsion.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include <visualization_msgs/msg/marker.hpp>

namespace tvvf_vo_c
{
namespace
{
inline int clamp_index(int value, int min_value, int max_value)
{
    return std::max(min_value, std::min(value, max_value));
}

inline int world_to_index(double world, double origin, double resolution)
{
    return static_cast<int>(std::floor((world - origin) / resolution));
}

inline double cell_center(double origin, double resolution, int index)
{
    return origin + (static_cast<double>(index) + 0.5) * resolution;
}
}  // namespace

std::vector<Position> extract_map_obstacle_positions(
    const nav_msgs::msg::OccupancyGrid& map,
    const FieldRegion& region,
    const Position& robot_pos,
    const MapRepulsionSettings& settings)
{
    std::vector<Position> obstacles;

    if (map.info.width == 0 || map.info.height == 0 || map.info.resolution <= 0.0) {
        return obstacles;
    }

    if (!region.is_valid()) {
        return obstacles;
    }

    const double resolution = map.info.resolution;
    const double origin_x = map.info.origin.position.x;
    const double origin_y = map.info.origin.position.y;
    const int width = static_cast<int>(map.info.width);
    const int height = static_cast<int>(map.info.height);

    const int stride = std::max(1, static_cast<int>(std::round(settings.sampling_step / resolution)));
    const double max_distance_sq = (settings.max_distance > 0.0)
        ? settings.max_distance * settings.max_distance
        : std::numeric_limits<double>::infinity();

    const int region_min_x = clamp_index(world_to_index(region.min.x, origin_x, resolution), 0, width - 1);
    const int region_max_x = clamp_index(world_to_index(region.max.x, origin_x, resolution), 0, width - 1);
    const int region_min_y = clamp_index(world_to_index(region.min.y, origin_y, resolution), 0, height - 1);
    const int region_max_y = clamp_index(world_to_index(region.max.y, origin_y, resolution), 0, height - 1);

    if (region_min_x > region_max_x || region_min_y > region_max_y) {
        return obstacles;
    }

    for (int y = region_min_y; y <= region_max_y; y += stride) {
        for (int x = region_min_x; x <= region_max_x; x += stride) {
            const int index = y * width + x;
            const int8_t occupancy = map.data[index];

            if (occupancy < settings.occupancy_threshold) {
                continue;
            }

            const double wx = cell_center(origin_x, resolution, x);
            const double wy = cell_center(origin_y, resolution, y);
            const double dx = wx - robot_pos.x;
            const double dy = wy - robot_pos.y;
            const double distance_sq = dx * dx + dy * dy;

            if (distance_sq > max_distance_sq) {
                continue;
            }

            obstacles.emplace_back(wx, wy);
        }
    }

    return obstacles;
}

visualization_msgs::msg::MarkerArray create_map_obstacle_markers(
    const std::vector<Position>& positions,
    double scale,
    const std::string& frame_id,
    const builtin_interfaces::msg::Time& stamp)
{
    visualization_msgs::msg::MarkerArray array;
    const double safe_scale = std::max(0.05, scale);

    int marker_id = 0;
    for (const auto& pos : positions) {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = frame_id;
        marker.header.stamp = stamp;
        marker.ns = "map_static_obstacles";
        marker.id = marker_id++;
        marker.type = visualization_msgs::msg::Marker::SPHERE;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.pose.position.x = pos.x;
        marker.pose.position.y = pos.y;
        marker.pose.position.z = 0.0;
        marker.scale.x = safe_scale;
        marker.scale.y = safe_scale;
        marker.scale.z = 0.1;
        marker.color.r = 1.0;
        marker.color.g = 0.4;
        marker.color.b = 0.0;
        marker.color.a = 0.6;
        array.markers.push_back(marker);
    }

    return array;
}

}  // namespace tvvf_vo_c
