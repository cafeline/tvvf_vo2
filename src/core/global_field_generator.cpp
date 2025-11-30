// src/core/global_field_generator.cpp

#include "tvvf_vo_c/core/global_field_generator.hpp"
#include <chrono>
#include <cmath>
#include <algorithm>
#include <optional>

namespace tvvf_vo_c {

// 定数定義
namespace {
    // 斥力計算パラメータ
    constexpr double REPULSIVE_GAIN = 1.0;            // 斥力ゲイン
    constexpr double INFLUENCE_RADIUS = 3.0;          // 影響半径[m]
    constexpr double MIN_DISTANCE = 0.1;              // 最小距離[m]（ゼロ除算防止）
    
    // ブレンディングパラメータ
    constexpr double DEFAULT_BLEND_WEIGHT = 0.5;      // デフォルトのブレンド比率
    constexpr double MIN_MAGNITUDE_THRESHOLD = 0.01;  // ベクトル大きさの最小閾値
    
    // パフォーマンス最適化
    constexpr double INFLUENCE_RADIUS_SQ = INFLUENCE_RADIUS * INFLUENCE_RADIUS;
    constexpr double MIN_DISTANCE_SQ = MIN_DISTANCE * MIN_DISTANCE;

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

    void stampDynamicObstacles(nav_msgs::msg::OccupancyGrid& map,
                               const std::vector<DynamicObstacle>& obstacles,
                               double inflation_margin) {
        if (map.info.resolution <= 0.0 || map.data.empty()) {
            return;
        }

        const double res = map.info.resolution;
        const double origin_x = map.info.origin.position.x;
        const double origin_y = map.info.origin.position.y;
        const int width = static_cast<int>(map.info.width);
        const int height = static_cast<int>(map.info.height);

        auto clamp_index = [](int v, int min_v, int max_v) {
            return std::max(min_v, std::min(v, max_v));
        };

        for (const auto& obs : obstacles) {
            const double inflated_radius = std::max(obs.radius + inflation_margin, res * 0.5);

            const double min_x = obs.position.x - inflated_radius;
            const double max_x = obs.position.x + inflated_radius;
            const double min_y = obs.position.y - inflated_radius;
            const double max_y = obs.position.y + inflated_radius;

            int min_ix = clamp_index(static_cast<int>(std::floor((min_x - origin_x) / res)), 0, width - 1);
            int max_ix = clamp_index(static_cast<int>(std::ceil((max_x - origin_x) / res)), 0, width - 1);
            int min_iy = clamp_index(static_cast<int>(std::floor((min_y - origin_y) / res)), 0, height - 1);
            int max_iy = clamp_index(static_cast<int>(std::ceil((max_y - origin_y) / res)), 0, height - 1);

            for (int y = min_iy; y <= max_iy; ++y) {
                for (int x = min_ix; x <= max_ix; ++x) {
                    const double cx = origin_x + (static_cast<double>(x) + 0.5) * res;
                    const double cy = origin_y + (static_cast<double>(y) + 0.5) * res;
                    const double dx = cx - obs.position.x;
                    const double dy = cy - obs.position.y;
                    if (std::hypot(dx, dy) <= inflated_radius) {
                        map.data[static_cast<size_t>(y) * width + x] = 100;
                    }
                }
            }
        }
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
    const std::vector<DynamicObstacle>& obstacles,
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

    nav_msgs::msg::OccupancyGrid map_with_obstacles = *map_ptr;
    if (!obstacles.empty()) {
        const double inflation = std::max(cost_map_settings_.clearance_epsilon, map_with_obstacles.info.resolution * 0.5);
        stampDynamicObstacles(map_with_obstacles, obstacles, inflation);
    }

    auto cost_map = cost_map_builder_.build(map_with_obstacles);
    if (!cost_map.isValid()) {
        static_field_computed_ = false;
        return VectorField();
    }

    last_cost_map_result_ = cost_map;

    fast_marching_->initializeFromOccupancyGrid(map_with_obstacles, cost_map.speed_layer);
    fast_marching_->computeDistanceField(goal);
    fast_marching_->generateVectorField();

    static_field_ = fast_marching_->getField();
    static_field_.region = applied_region;
    static_field_computed_ = true;

    auto end = std::chrono::high_resolution_clock::now();
    last_computation_time_ = std::chrono::duration<double>(end - start).count();

    return static_field_;
}

// 動的障害物を考慮したフィールド生成
VectorField GlobalFieldGenerator::generateField(const std::vector<DynamicObstacle>& obstacles) {
    auto start = std::chrono::high_resolution_clock::now();
    
    if (!static_field_computed_) {
        // 静的場が計算されていない場合は空のフィールドを返す
        return VectorField();
    }
    
    // 障害物がない場合は静的場をそのまま返す（最適化）
    if (obstacles.empty()) {
        auto end = std::chrono::high_resolution_clock::now();
        last_computation_time_ = std::chrono::duration<double>(end - start).count();
        return static_field_;
    }
    
    // 動的障害物の影響をブレンド
    VectorField result = blendWithDynamicObstacles(static_field_, obstacles);
    
    auto end = std::chrono::high_resolution_clock::now();
    last_computation_time_ = std::chrono::duration<double>(end - start).count();
    
    return result;
}

// 動的障害物の影響をブレンド
VectorField GlobalFieldGenerator::blendWithDynamicObstacles(const VectorField& static_field,
                                                            const std::vector<DynamicObstacle>& obstacles) {
    // フィールドをコピー
    VectorField blended_field = static_field;
    
    // 各グリッドセルに対して動的障害物の影響を計算
    for (int y = 0; y < blended_field.height; ++y) {
        for (int x = 0; x < blended_field.width; ++x) {
            Position cell_pos = blended_field.gridToWorld(x, y);
            
            // このセルに対する全障害物からの斥力を計算
            auto total_repulsive_force = computeTotalRepulsiveForce(cell_pos, obstacles);
            
            // 斥力が有意な場合のみブレンド
            if (hasSignificantMagnitude(total_repulsive_force)) {
                blended_field.vectors[y][x] = blendAndNormalizeVectors(
                    static_field.vectors[y][x],
                    total_repulsive_force,
                    DEFAULT_BLEND_WEIGHT
                );
            }
        }
    }
    
    return blended_field;
}

// 計算時間の取得
double GlobalFieldGenerator::getLastComputationTime() const {
    return last_computation_time_;
}

// Private helper functions

// 全障害物からの斥力の合計を計算
std::array<double, 2> GlobalFieldGenerator::computeTotalRepulsiveForce(
    const Position& position,
    const std::vector<DynamicObstacle>& obstacles) const {
    
    std::array<double, 2> total_force = {0.0, 0.0};
    
    for (const auto& obstacle : obstacles) {
        auto force = computeRepulsiveForceFromObstacle(position, obstacle);
        total_force[0] += force[0];
        total_force[1] += force[1];
    }
    
    return total_force;
}

// 単一障害物からの斥力を計算
std::array<double, 2> GlobalFieldGenerator::computeRepulsiveForceFromObstacle(
    const Position& position,
    const DynamicObstacle& obstacle) const {
    
    // 障害物への相対位置
    double dx = position.x - obstacle.position.x;
    double dy = position.y - obstacle.position.y;
    double distance_sq = dx * dx + dy * dy;
    
    // 影響範囲外の早期リターン（平方根計算を避ける）
    if (distance_sq > INFLUENCE_RADIUS_SQ || distance_sq < MIN_DISTANCE_SQ) {
        return {0.0, 0.0};
    }
    
    double distance = std::sqrt(distance_sq);
    
    // 斥力の大きさ（Coulomb型ポテンシャル）
    double force_magnitude = REPULSIVE_GAIN * (1.0 / distance - 1.0 / INFLUENCE_RADIUS);
    
    // 正規化された方向ベクトル × 力の大きさ
    return {
        force_magnitude * (dx / distance),
        force_magnitude * (dy / distance)
    };
}

// ベクトルのブレンドと正規化
std::array<double, 2> GlobalFieldGenerator::blendAndNormalizeVectors(
    const std::array<double, 2>& static_vector,
    const std::array<double, 2>& dynamic_vector,
    double blend_weight) const {
    
    // 線形補間によるブレンド
    std::array<double, 2> blended = {
        (1.0 - blend_weight) * static_vector[0] + blend_weight * dynamic_vector[0],
        (1.0 - blend_weight) * static_vector[1] + blend_weight * dynamic_vector[1]
    };
    
    // 正規化
    return normalizeVector(blended);
}

// ベクトルの正規化
std::array<double, 2> GlobalFieldGenerator::normalizeVector(
    const std::array<double, 2>& vector) const {
    
    double magnitude_sq = vector[0] * vector[0] + vector[1] * vector[1];
    
    // ゼロベクトルのチェック
    if (magnitude_sq < MIN_MAGNITUDE_THRESHOLD * MIN_MAGNITUDE_THRESHOLD) {
        return {0.0, 0.0};
    }
    
    double inv_magnitude = 1.0 / std::sqrt(magnitude_sq);
    return {
        vector[0] * inv_magnitude,
        vector[1] * inv_magnitude
    };
}

// ベクトルが有意な大きさを持つかチェック
bool GlobalFieldGenerator::hasSignificantMagnitude(
    const std::array<double, 2>& vector) const {
    
    return std::abs(vector[0]) > MIN_MAGNITUDE_THRESHOLD || 
           std::abs(vector[1]) > MIN_MAGNITUDE_THRESHOLD;
}

// ロボット位置での速度ベクトル取得
std::array<double, 2> GlobalFieldGenerator::getVelocityAt(const Position& position,
                                                          const std::vector<DynamicObstacle>& obstacles) {
    if (!static_field_computed_) {
        return {0.0, 0.0};
    }
    
    // 静的場からベクトル取得
    auto [grid_x, grid_y] = static_field_.worldToGrid(position);
    
    // 範囲チェック
    if (grid_x < 0 || grid_x >= static_field_.width || 
        grid_y < 0 || grid_y >= static_field_.height) {
        return {0.0, 0.0};
    }
    
    // 静的場のベクトル
    std::array<double, 2> static_vector = static_field_.vectors[grid_y][grid_x];
    
    // 動的障害物からの斥力
    if (!obstacles.empty()) {
        auto repulsive_force = computeTotalRepulsiveForce(position, obstacles);
        
        // 斥力が有意な場合のみブレンド
        if (hasSignificantMagnitude(repulsive_force)) {
            static_vector = blendAndNormalizeVectors(
                static_vector,
                repulsive_force,
                DEFAULT_BLEND_WEIGHT
            );
        }
    }
    
    return static_vector;
}

}  // namespace tvvf_vo_c
