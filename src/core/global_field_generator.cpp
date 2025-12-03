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

    std::array<double, 2> bilinearSampleVector(
        const VectorField& base_field,
        const Position& pos)
    {
        const auto [gx, gy] = base_field.worldToGrid(pos);
        const int x0 = gx;
        const int y0 = gy;
        const int x1 = x0 + 1;
        const int y1 = y0 + 1;

        if (x0 < 0 || y0 < 0 ||
            x1 >= base_field.width || y1 >= base_field.height) {
            return {0.0, 0.0};
        }

        const double wx = (pos.x - (base_field.origin.x + (static_cast<double>(x0) + 0.5) * base_field.resolution)) / base_field.resolution;
        const double wy = (pos.y - (base_field.origin.y + (static_cast<double>(y0) + 0.5) * base_field.resolution)) / base_field.resolution;
        const double sx = std::clamp(wx, 0.0, 1.0);
        const double sy = std::clamp(wy, 0.0, 1.0);

        const auto v00 = base_field.vectors[static_cast<size_t>(y0)][static_cast<size_t>(x0)];
        const auto v10 = base_field.vectors[static_cast<size_t>(y0)][static_cast<size_t>(x1)];
        const auto v01 = base_field.vectors[static_cast<size_t>(y1)][static_cast<size_t>(x0)];
        const auto v11 = base_field.vectors[static_cast<size_t>(y1)][static_cast<size_t>(x1)];

        const double vx0 = v00[0] * (1.0 - sx) + v10[0] * sx;
        const double vx1 = v01[0] * (1.0 - sx) + v11[0] * sx;
        const double vy0 = v00[1] * (1.0 - sx) + v10[1] * sx;
        const double vy1 = v01[1] * (1.0 - sx) + v11[1] * sx;

        const double vx = vx0 * (1.0 - sy) + vx1 * sy;
        const double vy = vy0 * (1.0 - sy) + vy1 * sy;
        return {vx, vy};
    }

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
      last_cost_map_result_(std::nullopt),
      multi_res_settings_() {
    fast_marching_ = std::make_unique<FastMarching>();
}

void GlobalFieldGenerator::setCostMapSettings(const CostMapSettings& settings) {
    cost_map_settings_ = settings;
    cost_map_builder_ = CostMapBuilder(cost_map_settings_);
    if (fast_marching_) {
        fast_marching_->setOccupancyThresholds(
            cost_map_settings_.occupied_threshold,
            cost_map_settings_.free_threshold);
    }
}

// 静的場の事前計算
void GlobalFieldGenerator::precomputeStaticField(const nav_msgs::msg::OccupancyGrid& map,
                                                  const Position& goal,
                                                  const std::optional<FieldRegion>& region,
                                                  const std::optional<Position>& focus) {
    computeFieldOnTheFly(map, goal, region, focus);
}

VectorField GlobalFieldGenerator::computeFieldOnTheFly(
    const nav_msgs::msg::OccupancyGrid& map,
    const Position& goal,
    const std::optional<FieldRegion>& region,
    const std::optional<Position>& focus) {

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

    const nav_msgs::msg::OccupancyGrid* overlay_source = map_ptr;

    nav_msgs::msg::OccupancyGrid resampled_global;
    if (multi_res_settings_.global_resolution > 1e-6 &&
        std::abs(multi_res_settings_.global_resolution - map_ptr->info.resolution) > 1e-6)
    {
        resampled_global = resampleOccupancyGrid(*map_ptr, multi_res_settings_.global_resolution);
        map_ptr = &resampled_global;
    }

    if (map_ptr->info.width == 0 || map_ptr->info.height == 0) {
        static_field_computed_ = false;
        return VectorField();
    }

    if (!last_cost_map_result_) {
        last_cost_map_result_.emplace();
    }
    cost_map_builder_.build(*map_ptr, *last_cost_map_result_);
    last_cost_map_result_->overlays.clear();
    if (!last_cost_map_result_->isValid()) {
        last_cost_map_result_.reset();
        static_field_computed_ = false;
        return VectorField();
    }

    fast_marching_->initializeFromOccupancyGrid(*map_ptr, last_cost_map_result_->speed_layer);
    fast_marching_->computeDistanceField(goal);
    fast_marching_->generateVectorField();

    static_field_ = fast_marching_->getField();
    static_field_.region = applied_region;
    static_field_.clearOverlays();

    if (multi_res_settings_.enabled() && focus.has_value()) {
        if (auto local_patch = buildLocalPatch(*overlay_source, focus.value())) {
            CostMapResult local_cost;
            cost_map_builder_.build(*local_patch, local_cost);
            if (local_cost.isValid()) {
                last_cost_map_result_->overlays.push_back(local_cost.toLayer());

                const FieldRegion patch_region(
                    Position(local_patch->info.origin.position.x, local_patch->info.origin.position.y),
                    Position(
                        local_patch->info.origin.position.x +
                            static_cast<double>(local_patch->info.width) * local_patch->info.resolution,
                        local_patch->info.origin.position.y +
                            static_cast<double>(local_patch->info.height) * local_patch->info.resolution));

                const bool goal_inside = patch_region.contains(goal);
                if (goal_inside) {
                    FastMarching local_fmm;
                    local_fmm.setOccupancyThresholds(
                        cost_map_settings_.occupied_threshold,
                        cost_map_settings_.free_threshold);
                    local_fmm.initializeFromOccupancyGrid(*local_patch, local_cost.speed_layer);
                    local_fmm.computeDistanceField(goal);
                    local_fmm.generateVectorField();

                    VectorField local_field = local_fmm.getField();
                    local_field.region = patch_region;
                    static_field_.addOverlay(local_field);
                } else {
                    // ゴールがパッチ外なら、ベースベクトルを高解像度にバイリニア補間して重ねる
                    VectorField refined;
                    const int up_width = static_cast<int>(local_patch->info.width);
                    const int up_height = static_cast<int>(local_patch->info.height);
                    refined.resize(up_width, up_height);
                    refined.resolution = local_patch->info.resolution;
                    refined.origin = Position(
                        local_patch->info.origin.position.x,
                        local_patch->info.origin.position.y);
                    refined.region = patch_region;

                    for (int y = 0; y < up_height; ++y) {
                        for (int x = 0; x < up_width; ++x) {
                            const Position wp = refined.gridToWorld(x, y);
                            auto v = bilinearSampleVector(static_field_, wp);
                            const double norm = std::hypot(v[0], v[1]);
                            if (norm > 1e-9) {
                                v[0] /= norm;
                                v[1] /= norm;
                            } else {
                                v = {0.0, 0.0};
                            }
                            refined.vectors[static_cast<size_t>(y)][static_cast<size_t>(x)] = v;
                        }
                    }
                    static_field_.addOverlay(refined);
                }
            }
        }
    }

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

    return static_field_.getVector(position);
}

nav_msgs::msg::OccupancyGrid GlobalFieldGenerator::resampleOccupancyGrid(
    const nav_msgs::msg::OccupancyGrid& src,
    double new_resolution) const
{
    nav_msgs::msg::OccupancyGrid out = src;
    if (new_resolution <= 0.0 ||
        std::abs(new_resolution - src.info.resolution) < 1e-9)
    {
        return out;
    }

    const double size_x = static_cast<double>(src.info.width) * src.info.resolution;
    const double size_y = static_cast<double>(src.info.height) * src.info.resolution;
    const uint32_t new_width = static_cast<uint32_t>(std::ceil(size_x / new_resolution));
    const uint32_t new_height = static_cast<uint32_t>(std::ceil(size_y / new_resolution));

    out.info.resolution = new_resolution;
    out.info.width = new_width;
    out.info.height = new_height;
    out.data.assign(static_cast<size_t>(new_width) * new_height, -1);

    const double origin_x = src.info.origin.position.x;
    const double origin_y = src.info.origin.position.y;
    const double src_res = src.info.resolution;

    for (uint32_t row = 0; row < new_height; ++row) {
        for (uint32_t col = 0; col < new_width; ++col) {
            const double wx = origin_x + (static_cast<double>(col) + 0.5) * new_resolution;
            const double wy = origin_y + (static_cast<double>(row) + 0.5) * new_resolution;
            const int src_col = static_cast<int>(std::floor((wx - origin_x) / src_res));
            const int src_row = static_cast<int>(std::floor((wy - origin_y) / src_res));
            if (src_col < 0 || src_row < 0 ||
                src_col >= static_cast<int>(src.info.width) ||
                src_row >= static_cast<int>(src.info.height)) {
                continue;
            }
            const size_t src_idx =
              static_cast<size_t>(src_row) * src.info.width + static_cast<size_t>(src_col);
            out.data[static_cast<size_t>(row) * new_width + static_cast<size_t>(col)] =
              src.data[src_idx];
        }
    }
    return out;
}

std::optional<nav_msgs::msg::OccupancyGrid> GlobalFieldGenerator::buildLocalPatch(
    const nav_msgs::msg::OccupancyGrid& map,
    const Position& focus) const
{
    if (!multi_res_settings_.enabled()) {
        return std::nullopt;
    }
    if (map.info.width == 0 || map.info.height == 0) {
        return std::nullopt;
    }

    const double radius = multi_res_settings_.local_radius;
    const double patch_res = multi_res_settings_.local_resolution;
    FieldRegion desired(
        Position(focus.x - radius, focus.y - radius),
        Position(focus.x + radius, focus.y + radius));

    const double map_res = map.info.resolution;
    FieldRegion map_bounds(
        Position(map.info.origin.position.x, map.info.origin.position.y),
        Position(
            map.info.origin.position.x + static_cast<double>(map.info.width) * map_res,
            map.info.origin.position.y + static_cast<double>(map.info.height) * map_res));

    FieldRegion clamped;
    clamped.min.x = std::max(desired.min.x, map_bounds.min.x);
    clamped.min.y = std::max(desired.min.y, map_bounds.min.y);
    clamped.max.x = std::min(desired.max.x, map_bounds.max.x);
    clamped.max.y = std::min(desired.max.y, map_bounds.max.y);

    if (!clamped.is_valid() || clamped.min.x >= clamped.max.x || clamped.min.y >= clamped.max.y) {
        return std::nullopt;
    }

    nav_msgs::msg::OccupancyGrid patch;
    patch.info.resolution = patch_res;
    patch.info.origin.position.x = clamped.min.x;
    patch.info.origin.position.y = clamped.min.y;
    patch.info.width = static_cast<uint32_t>(
        std::ceil((clamped.max.x - clamped.min.x) / patch_res));
    patch.info.height = static_cast<uint32_t>(
        std::ceil((clamped.max.y - clamped.min.y) / patch_res));
    patch.data.assign(static_cast<size_t>(patch.info.width) * patch.info.height, -1);

    const double origin_x = map.info.origin.position.x;
    const double origin_y = map.info.origin.position.y;
    for (uint32_t row = 0; row < patch.info.height; ++row) {
        for (uint32_t col = 0; col < patch.info.width; ++col) {
            const double wx = patch.info.origin.position.x +
                (static_cast<double>(col) + 0.5) * patch_res;
            const double wy = patch.info.origin.position.y +
                (static_cast<double>(row) + 0.5) * patch_res;
            const int src_col = static_cast<int>(std::floor((wx - origin_x) / map_res));
            const int src_row = static_cast<int>(std::floor((wy - origin_y) / map_res));
            if (src_col < 0 || src_row < 0 ||
                src_col >= static_cast<int>(map.info.width) ||
                src_row >= static_cast<int>(map.info.height)) {
                continue;
            }
            const size_t src_idx =
              static_cast<size_t>(src_row) * map.info.width + static_cast<size_t>(src_col);
            patch.data[static_cast<size_t>(row) * patch.info.width + static_cast<size_t>(col)] =
              map.data[src_idx];
        }
    }

    return patch;
}

}  // namespace tvvf_vo_c
