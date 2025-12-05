//
// cost_map_builder.cpp
//

#include "tvvf_vo_c/core/cost_map_builder.hpp"
#include <algorithm>
#include <queue>
#include <limits>
#include <cmath>
#include <functional>

namespace tvvf_vo_c {

namespace {
struct QueueNode {
    double distance;
    int x;
    int y;

    bool operator>(const QueueNode& other) const {
        return distance > other.distance;
    }
};

inline bool is_obstacle_cell(int8_t occupancy,
                             double occupied_threshold,
                             double free_threshold) {
    if (occupancy < 0) {
        return true;
    }
    if (occupancy >= occupied_threshold) {
        return true;
    }
    return occupancy > free_threshold && occupancy < occupied_threshold;
}
}  // namespace

CostMapBuilder::CostMapBuilder(const CostMapSettings& settings)
    : settings_(settings) {}

void CostMapBuilder::build(const nav_msgs::msg::OccupancyGrid& map,
                           CostMapResult& result) const {
    result.width = map.info.width;
    result.height = map.info.height;
    result.resolution = map.info.resolution;

    if (result.width == 0 || result.height == 0) {
        result.speed_layer.clear();
        result.clearance_layer.clear();
        return;
    }

    const size_t cell_count = static_cast<size_t>(result.width) * result.height;
    if (result.speed_layer.size() != cell_count) {
        result.speed_layer.resize(cell_count);
    }
    if (result.clearance_layer.size() != cell_count) {
        result.clearance_layer.resize(cell_count);
    }
    if (obstacle_mask_buffer_.size() != cell_count) {
        obstacle_mask_buffer_.resize(cell_count);
    }
    std::fill(result.speed_layer.begin(), result.speed_layer.end(), settings_.max_speed);
    std::fill(result.clearance_layer.begin(), result.clearance_layer.end(), std::numeric_limits<double>::infinity());
    std::fill(obstacle_mask_buffer_.begin(), obstacle_mask_buffer_.end(), false);

    const double resolution = std::max(result.resolution, 1e-6);
    const double orthogonal_step = resolution;
    const double diagonal_step = std::sqrt(2.0) * resolution;

    std::priority_queue<QueueNode, std::vector<QueueNode>, std::greater<QueueNode>> queue;

    for (uint32_t y = 0; y < result.height; ++y) {
        for (uint32_t x = 0; x < result.width; ++x) {
            const size_t index = static_cast<size_t>(y) * result.width + x;
            const int8_t occupancy = map.data[index];
            if (is_obstacle_cell(occupancy, settings_.occupied_threshold, settings_.free_threshold)) {
                obstacle_mask_buffer_[index] = true;
                result.clearance_layer[index] = 0.0;
                queue.push(QueueNode{0.0, static_cast<int>(x), static_cast<int>(y)});
            }
        }
    }

    if (!queue.empty()) {
        static constexpr int DX[8] = {-1, 0, 1, -1, 1, -1, 0, 1};
        static constexpr int DY[8] = {-1, -1, -1, 0, 0, 1, 1, 1};
        const double STEPS[8] = {
            diagonal_step, orthogonal_step, diagonal_step,
            orthogonal_step, orthogonal_step,
            diagonal_step, orthogonal_step, diagonal_step
        };

        while (!queue.empty()) {
            const QueueNode node = queue.top();
            queue.pop();

            if (node.distance > result.clearance_layer[static_cast<size_t>(node.y) * result.width + node.x]) {
                continue;
            }

            for (int i = 0; i < 8; ++i) {
                const int nx = node.x + DX[i];
                const int ny = node.y + DY[i];
                if (nx < 0 || ny < 0 ||
                    nx >= static_cast<int>(result.width) ||
                    ny >= static_cast<int>(result.height)) {
                    continue;
                }

                const size_t neighbor_index = static_cast<size_t>(ny) * result.width + nx;
                if (obstacle_mask_buffer_[neighbor_index]) {
                    continue;
                }

                const double new_distance = node.distance + STEPS[i];
                if (new_distance < result.clearance_layer[neighbor_index]) {
                    result.clearance_layer[neighbor_index] = new_distance;
                    queue.push(QueueNode{new_distance, nx, ny});
                }
            }
        }
    } else {
        std::fill(result.clearance_layer.begin(), result.clearance_layer.end(), settings_.max_clearance);
    }

    const double max_clearance = std::max(settings_.max_clearance, resolution);
    const double clearance_epsilon = std::max(settings_.clearance_epsilon, resolution * 0.25);
    const double min_speed = std::max(0.0, settings_.min_speed);
    const double max_speed = std::max(settings_.max_speed, min_speed);

    for (size_t idx = 0; idx < cell_count; ++idx) {
        if (obstacle_mask_buffer_[idx]) {
            result.speed_layer[idx] = 0.0;
            continue;
        }

        double clearance = result.clearance_layer[idx];
        if (!std::isfinite(clearance)) {
            clearance = max_clearance;
        }
        clearance = std::clamp(clearance, 0.0, max_clearance);
        result.clearance_layer[idx] = clearance;

        if (clearance <= clearance_epsilon) {
            result.speed_layer[idx] = 0.0;
            continue;
        }

        const double limited_clearance = std::max(clearance, clearance_epsilon);
        double raw_speed = 1.0 / (1.0 + settings_.alpha / limited_clearance);
        raw_speed = std::clamp(raw_speed, min_speed, max_speed);
        result.speed_layer[idx] = raw_speed;
    }
}

CostMapResult CostMapBuilder::build(const nav_msgs::msg::OccupancyGrid& map) const {
    CostMapResult result;
    build(map, result);
    return result;
}

void CostMapBuilder::apply_escape_speed(const nav_msgs::msg::OccupancyGrid& map,
                                        const Position& robot_position,
                                        CostMapResult& result,
                                        double escape_radius,
                                        double escape_speed_min) const
{
    const size_t expected_size = static_cast<size_t>(result.width) * result.height;
    if (!result.isValid() || result.speed_layer.size() != expected_size) {
        return;
    }
    if (map.info.width != result.width || map.info.height != result.height) {
        return;
    }
    const double radius_sq = escape_radius * escape_radius;
    if (radius_sq <= 0.0 || expected_size == 0) {
        return;
    }

    const double resolution = std::max(result.resolution, 1e-6);
    const double origin_x = map.info.origin.position.x;
    const double origin_y = map.info.origin.position.y;
    const double escape_speed = std::max(settings_.min_speed, escape_speed_min);
    const bool has_obstacle_mask = obstacle_mask_buffer_.size() == expected_size;

    for (uint32_t row = 0; row < result.height; ++row) {
        for (uint32_t col = 0; col < result.width; ++col) {
            const size_t idx = static_cast<size_t>(row) * result.width + col;
            if (has_obstacle_mask && obstacle_mask_buffer_[idx]) {
                continue;
            }

            const double wx = origin_x + (static_cast<double>(col) + 0.5) * resolution;
            const double wy = origin_y + (static_cast<double>(row) + 0.5) * resolution;
            const double dx = wx - robot_position.x;
            const double dy = wy - robot_position.y;
            if (dx * dx + dy * dy <= radius_sq) {
                result.speed_layer[idx] = escape_speed;
            }
        }
    }
}

}  // namespace tvvf_vo_c
