// src/core/global_field_generator.cpp

#include "tvvf_vo_c/core/global_field_generator.hpp"
#include <chrono>
#include <cmath>
#include <algorithm>
#include <optional>

namespace tvvf_vo_c {

// 定数定義
namespace {
    struct CroppedMapResult {
        nav_msgs::msg::OccupancyGrid map;
        FieldRegion region;
    };

    std::optional<CroppedMapResult> cropOccupancyGrid(
        const nav_msgs::msg::OccupancyGrid& map,
        const FieldRegion& requested_region) {
        
        if (!requested_region.is_valid() || map.info.width == 0 || map.info.height == 0) {
            return std::nullopt;
        }

        const double resolution = map.info.resolution;
        const double origin_x = map.info.origin.position.x;
        const double origin_y = map.info.origin.position.y;
        const int map_width = static_cast<int>(map.info.width);
        const int map_height = static_cast<int>(map.info.height);

        int min_x_index = static_cast<int>(std::floor((requested_region.min.x - origin_x) / resolution));
        int max_x_index = static_cast<int>(std::ceil((requested_region.max.x - origin_x) / resolution)) - 1;
        int min_y_index = static_cast<int>(std::floor((requested_region.min.y - origin_y) / resolution));
        int max_y_index = static_cast<int>(std::ceil((requested_region.max.y - origin_y) / resolution)) - 1;

        if (max_x_index < 0 || min_x_index >= map_width ||
            max_y_index < 0 || min_y_index >= map_height) {
            return std::nullopt;
        }

        min_x_index = std::clamp(min_x_index, 0, map_width - 1);
        max_x_index = std::clamp(max_x_index, 0, map_width - 1);
        min_y_index = std::clamp(min_y_index, 0, map_height - 1);
        max_y_index = std::clamp(max_y_index, 0, map_height - 1);

        if (min_x_index > max_x_index || min_y_index > max_y_index) {
            return std::nullopt;
        }

        nav_msgs::msg::OccupancyGrid submap = map;
        submap.info.width = static_cast<uint32_t>(max_x_index - min_x_index + 1);
        submap.info.height = static_cast<uint32_t>(max_y_index - min_y_index + 1);
        submap.info.origin.position.x = origin_x + min_x_index * resolution;
        submap.info.origin.position.y = origin_y + min_y_index * resolution;
        submap.data.resize(submap.info.width * submap.info.height);

        for (uint32_t y = 0; y < submap.info.height; ++y) {
            for (uint32_t x = 0; x < submap.info.width; ++x) {
                const int original_x = static_cast<int>(min_x_index + x);
                const int original_y = static_cast<int>(min_y_index + y);
                submap.data[y * submap.info.width + x] =
                    map.data[original_y * map_width + original_x];
            }
        }

        const double aligned_min_x = submap.info.origin.position.x;
        const double aligned_min_y = submap.info.origin.position.y;
        const double aligned_max_x = aligned_min_x + submap.info.width * resolution;
        const double aligned_max_y = aligned_min_y + submap.info.height * resolution;

        return CroppedMapResult{
            submap,
            FieldRegion(
                Position(aligned_min_x, aligned_min_y),
                Position(aligned_max_x, aligned_max_y)
            )
        };
    }

}

// コンストラクタ
GlobalFieldGenerator::GlobalFieldGenerator() 
    : last_computation_time_(0.0),
      static_field_computed_(false),
      cost_map_settings_(),
      cost_map_builder_(cost_map_settings_),
      last_cost_map_result_(std::nullopt) {
    fast_marching_ = std::make_unique<FastMarching>();
}

void GlobalFieldGenerator::setCostMapSettings(const CostMapSettings& settings) {
    cost_map_settings_ = settings;
    cost_map_builder_ = CostMapBuilder(cost_map_settings_);
}

// 静的場の事前計算
void GlobalFieldGenerator::precomputeStaticField(const nav_msgs::msg::OccupancyGrid& map,
                                                  const Position& goal,
                                                  const std::optional<FieldRegion>& region) {
    nav_msgs::msg::OccupancyGrid cropped_map;
    const nav_msgs::msg::OccupancyGrid* map_ptr = &map;
    std::optional<FieldRegion> applied_region = std::nullopt;

    if (region.has_value()) {
        if (auto cropped = cropOccupancyGrid(map, region.value())) {
            cropped_map = std::move(cropped->map);
            applied_region = cropped->region;
            map_ptr = &cropped_map;
        }
    }

    auto cost_map = cost_map_builder_.build(*map_ptr);
    if (!cost_map.isValid()) {
        static_field_computed_ = false;
        return;
    }

    last_cost_map_result_ = cost_map;

    // Fast Marching Methodを使用して静的場を計算
    fast_marching_->initializeFromOccupancyGrid(*map_ptr, last_cost_map_result_->speed_layer);
    fast_marching_->computeDistanceField(goal);
    fast_marching_->generateVectorField();
    
    // 静的場を保存
    static_field_ = fast_marching_->getField();
    static_field_.region = applied_region;
    static_field_computed_ = true;
}

VectorField GlobalFieldGenerator::computeFieldOnTheFly(
    const nav_msgs::msg::OccupancyGrid& map,
    const Position& goal,
    const std::optional<FieldRegion>& region) {

    auto start = std::chrono::high_resolution_clock::now();

    nav_msgs::msg::OccupancyGrid cropped_map;
    const nav_msgs::msg::OccupancyGrid* map_ptr = &map;
    std::optional<FieldRegion> applied_region = std::nullopt;

    if (region.has_value()) {
        if (auto cropped = cropOccupancyGrid(map, region.value())) {
            cropped_map = std::move(cropped->map);
            applied_region = cropped->region;
            map_ptr = &cropped_map;
        }
    }

    if (map_ptr->info.width == 0 || map_ptr->info.height == 0) {
        static_field_computed_ = false;
        return VectorField();
    }

    auto cost_map = cost_map_builder_.build(*map_ptr);
    if (!cost_map.isValid()) {
        static_field_computed_ = false;
        return VectorField();
    }

    last_cost_map_result_ = cost_map;

    fast_marching_->initializeFromOccupancyGrid(*map_ptr, cost_map.speed_layer);
    fast_marching_->computeDistanceField(goal);
    fast_marching_->generateVectorField();

    static_field_ = fast_marching_->getField();
    static_field_.region = applied_region;
    static_field_computed_ = true;

    auto end = std::chrono::high_resolution_clock::now();
    last_computation_time_ = std::chrono::duration<double>(end - start).count();

    return static_field_;
}

// 計算時間の取得
double GlobalFieldGenerator::getLastComputationTime() const {
    return last_computation_time_;
}

std::array<double, 2> GlobalFieldGenerator::getVelocityAt(const Position& position) {
    if (!static_field_computed_) {
        return {0.0, 0.0};
    }

    auto [grid_x, grid_y] = static_field_.worldToGrid(position);
    if (grid_x < 0 || grid_x >= static_field_.width ||
        grid_y < 0 || grid_y >= static_field_.height) {
        return {0.0, 0.0};
    }

    return static_field_.vectors[grid_y][grid_x];
}

}  // namespace tvvf_vo_c
