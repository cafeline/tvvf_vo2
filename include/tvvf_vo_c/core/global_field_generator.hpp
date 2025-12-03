// include/tvvf_vo_c/core/global_field_generator.hpp

#ifndef TVVF_VO_C_CORE_GLOBAL_FIELD_GENERATOR_HPP_
#define TVVF_VO_C_CORE_GLOBAL_FIELD_GENERATOR_HPP_

#include "tvvf_vo_c/core/field_types.hpp"
#include "tvvf_vo_c/core/fast_marching.hpp"
#include "tvvf_vo_c/core/region_utils.hpp"
#include "tvvf_vo_c/core/cost_map_builder.hpp"
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <memory>
#include <vector>
#include <optional>

namespace tvvf_vo_c {

class GlobalFieldGenerator {
public:
    GlobalFieldGenerator();
    ~GlobalFieldGenerator() = default;

    struct MultiResolutionSettings {
        double global_resolution{0.0};
        double local_resolution{0.0};
        double local_radius{0.0};

        bool enabled() const {
            return local_resolution > 0.0 && local_radius > 0.0;
        }
    };

    void setCostMapSettings(const CostMapSettings& settings);
    void setMultiResolutionSettings(const MultiResolutionSettings& settings) { multi_res_settings_ = settings; }
    const std::optional<CostMapResult>& getLastCostMap() const { return last_cost_map_result_; }
    
    // 静的ベクトル場の事前計算
    void precomputeStaticField(const nav_msgs::msg::OccupancyGrid& map,
                                const Position& goal,
                                const std::optional<FieldRegion>& region = std::nullopt,
                                const std::optional<Position>& focus = std::nullopt);
    
    // 毎フレームのオンデマンド生成（静的事前計算なし）
    VectorField computeFieldOnTheFly(const nav_msgs::msg::OccupancyGrid& map,
                                     const Position& goal,
                                     const std::optional<FieldRegion>& region = std::nullopt,
                                     const std::optional<Position>& focus = std::nullopt);
    
    // パフォーマンス計測用
    double getLastComputationTime() const;
    
    // ロボット位置での速度ベクトル取得
    std::array<double, 2> getVelocityAt(const Position& position);
    
    // 静的場が計算済みかチェック
    bool isStaticFieldReady() const { return static_field_computed_; }
    
private:
    std::unique_ptr<FastMarching> fast_marching_;
    VectorField static_field_;
    double last_computation_time_;
    bool static_field_computed_;
    CostMapSettings cost_map_settings_;
    CostMapBuilder cost_map_builder_;
    std::optional<CostMapResult> last_cost_map_result_;
    MultiResolutionSettings multi_res_settings_;

    nav_msgs::msg::OccupancyGrid resampleOccupancyGrid(
        const nav_msgs::msg::OccupancyGrid& src, double new_resolution) const;
    std::optional<nav_msgs::msg::OccupancyGrid> buildLocalPatch(
        const nav_msgs::msg::OccupancyGrid& map,
        const Position& focus) const;
};

}  // namespace tvvf_vo_c

#endif  // TVVF_VO_C_CORE_GLOBAL_FIELD_GENERATOR_HPP_
