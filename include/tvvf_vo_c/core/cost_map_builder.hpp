//
// cost_map_builder.hpp
//

#ifndef TVVF_VO_C_CORE_COST_MAP_BUILDER_HPP_
#define TVVF_VO_C_CORE_COST_MAP_BUILDER_HPP_

#include <nav_msgs/msg/occupancy_grid.hpp>
#include "tvvf_vo_c/core/types.hpp"
#include <cstdint>
#include <vector>

namespace tvvf_vo_c {

struct CostMapSettings {
    double occupied_threshold{50.0};
    double free_threshold{0.0};
    double alpha{1.0};
    double min_speed{0.05};
    double max_speed{1.0};
    double clearance_epsilon{0.1};
    double max_clearance{5.0};
    double escape_speed_min{0.05};
};

struct CostMapResult {
    uint32_t width{0};
    uint32_t height{0};
    double resolution{0.0};
    std::vector<double> speed_layer;
    std::vector<double> clearance_layer;

    bool isValid() const {
        return width > 0 && height > 0 &&
               speed_layer.size() == width * height &&
               clearance_layer.size() == width * height;
    }

    double speedAt(int x, int y) const {
        if (!isValid() || x < 0 || y < 0 ||
            x >= static_cast<int>(width) || y >= static_cast<int>(height)) {
            return 0.0;
        }
        const size_t index = static_cast<size_t>(y) * width + x;
        return speed_layer[index];
    }

    double clearanceAt(int x, int y) const {
        if (!isValid() || x < 0 || y < 0 ||
            x >= static_cast<int>(width) || y >= static_cast<int>(height)) {
            return 0.0;
        }
        const size_t index = static_cast<size_t>(y) * width + x;
        return clearance_layer[index];
    }
};

class CostMapBuilder {
public:
    explicit CostMapBuilder(const CostMapSettings& settings = CostMapSettings());

    void build(const nav_msgs::msg::OccupancyGrid& map, CostMapResult& result) const;
    CostMapResult build(const nav_msgs::msg::OccupancyGrid& map) const;
    void apply_escape_speed(const nav_msgs::msg::OccupancyGrid& map,
                            const Position& robot_position,
                            CostMapResult& result,
                            double escape_radius,
                            double escape_speed_min) const;

private:
    CostMapSettings settings_;
    mutable std::vector<bool> obstacle_mask_buffer_;
};

}  // namespace tvvf_vo_c

#endif  // TVVF_VO_C_CORE_COST_MAP_BUILDER_HPP_
