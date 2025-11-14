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

CostMapResult CostMapBuilder::build(const nav_msgs::msg::OccupancyGrid& map) const {
    CostMapResult result;
    result.width = map.info.width;
    result.height = map.info.height;
    result.resolution = map.info.resolution;

    if (result.width == 0 || result.height == 0) {
        return result;
    }

    const size_t cell_count = static_cast<size_t>(result.width) * result.height;
    result.speed_layer.assign(cell_count, settings_.max_speed);
    result.clearance_layer.assign(cell_count, std::numeric_limits<double>::infinity());
    std::vector<bool> obstacle_mask(cell_count, false);

    const double resolution = std::max(result.resolution, 1e-6);
    const double orthogonal_step = resolution;
    const double diagonal_step = std::sqrt(2.0) * resolution;

    std::priority_queue<QueueNode, std::vector<QueueNode>, std::greater<QueueNode>> queue;

    for (uint32_t y = 0; y < result.height; ++y) {
        for (uint32_t x = 0; x < result.width; ++x) {
            const size_t index = static_cast<size_t>(y) * result.width + x;
            const int8_t occupancy = map.data[index];
            if (is_obstacle_cell(occupancy, settings_.occupied_threshold, settings_.free_threshold)) {
                obstacle_mask[index] = true;
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
                if (obstacle_mask[neighbor_index]) {
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
        if (obstacle_mask[idx]) {
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

    return result;
}

}  // namespace tvvf_vo_c
