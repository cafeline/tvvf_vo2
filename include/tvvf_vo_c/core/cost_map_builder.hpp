//
// cost_map_builder.hpp
//

#ifndef TVVF_VO_C_CORE_COST_MAP_BUILDER_HPP_
#define TVVF_VO_C_CORE_COST_MAP_BUILDER_HPP_

#include <nav_msgs/msg/occupancy_grid.hpp>
#include <cstdint>
#include <vector>
#include <limits>
#include <cmath>
#include "tvvf_vo_c/core/field_types.hpp"

namespace tvvf_vo_c {

struct CostMapSettings {
    double occupied_threshold{50.0};
    double free_threshold{0.0};
    double alpha{1.0};
    double min_speed{0.05};
    double max_speed{1.0};
    double clearance_epsilon{0.1};
    double max_clearance{5.0};
};

struct CostMapLayer {
    uint32_t width{0};
    uint32_t height{0};
    double resolution{0.0};
    Position origin{0.0, 0.0};
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

    FieldRegion region() const {
        return FieldRegion(
            origin,
            Position(
                origin.x + static_cast<double>(width) * resolution,
                origin.y + static_cast<double>(height) * resolution));
    }

    double speedAtWorld(const Position& pos) const {
        const auto [gx, gy] = worldToGrid(pos);
        return speedAt(gx, gy);
    }

    double clearanceAtWorld(const Position& pos) const {
        const auto [gx, gy] = worldToGrid(pos);
        return clearanceAt(gx, gy);
    }

    std::pair<int, int> worldToGrid(const Position& pos) const {
        const double inv_res = (resolution > 1e-9) ? 1.0 / resolution : 0.0;
        const int x = static_cast<int>(std::floor((pos.x - origin.x) * inv_res));
        const int y = static_cast<int>(std::floor((pos.y - origin.y) * inv_res));
        return {x, y};
    }
};

struct CostMapResult {
    uint32_t width{0};
    uint32_t height{0};
    double resolution{0.0};
    Position origin{0.0, 0.0};
    std::vector<double> speed_layer;
    std::vector<double> clearance_layer;
    std::vector<CostMapLayer> overlays;

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

    FieldRegion region() const {
        return FieldRegion(
            origin,
            Position(
                origin.x + static_cast<double>(width) * resolution,
                origin.y + static_cast<double>(height) * resolution));
    }

    CostMapLayer toLayer() const {
        CostMapLayer layer;
        layer.width = width;
        layer.height = height;
        layer.resolution = resolution;
        layer.origin = origin;
        layer.speed_layer = speed_layer;
        layer.clearance_layer = clearance_layer;
        return layer;
    }

    double speedAtWorld(const Position& pos) const {
        const CostMapLayer* best_layer = nullptr;
        double best_resolution = std::numeric_limits<double>::infinity();

        for (const auto& overlay : overlays) {
            if (!overlay.isValid()) {
                continue;
            }
            const auto overlay_region = overlay.region();
            if (!overlay_region.contains(pos)) {
                continue;
            }
            if (overlay.resolution < best_resolution) {
                best_layer = &overlay;
                best_resolution = overlay.resolution;
            }
        }

        if (best_layer) {
            return best_layer->speedAtWorld(pos);
        }

        const auto [gx, gy] = worldToGrid(pos);
        return speedAt(gx, gy);
    }

    double clearanceAtWorld(const Position& pos) const {
        const CostMapLayer* best_layer = nullptr;
        double best_resolution = std::numeric_limits<double>::infinity();

        for (const auto& overlay : overlays) {
            if (!overlay.isValid()) {
                continue;
            }
            const auto overlay_region = overlay.region();
            if (!overlay_region.contains(pos)) {
                continue;
            }
            if (overlay.resolution < best_resolution) {
                best_layer = &overlay;
                best_resolution = overlay.resolution;
            }
        }

        if (best_layer) {
            return best_layer->clearanceAtWorld(pos);
        }

        const auto [gx, gy] = worldToGrid(pos);
        return clearanceAt(gx, gy);
    }

    std::pair<int, int> worldToGrid(const Position& pos) const {
        const double inv_res = (resolution > 1e-9) ? 1.0 / resolution : 0.0;
        const int x = static_cast<int>(std::floor((pos.x - origin.x) * inv_res));
        const int y = static_cast<int>(std::floor((pos.y - origin.y) * inv_res));
        return {x, y};
    }
};

class CostMapBuilder {
public:
    explicit CostMapBuilder(const CostMapSettings& settings = CostMapSettings());

    void build(const nav_msgs::msg::OccupancyGrid& map, CostMapResult& result) const;
    CostMapResult build(const nav_msgs::msg::OccupancyGrid& map) const;

private:
    CostMapSettings settings_;
    mutable std::vector<bool> obstacle_mask_buffer_;
};

}  // namespace tvvf_vo_c

#endif  // TVVF_VO_C_CORE_COST_MAP_BUILDER_HPP_
