#ifndef TVVF_VO_C_CORE_MAP_REPULSION_HPP_
#define TVVF_VO_C_CORE_MAP_REPULSION_HPP_

#include <cstdint>
#include <vector>

#include <nav_msgs/msg/occupancy_grid.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <builtin_interfaces/msg/time.hpp>

#include "tvvf_vo_c/core/field_types.hpp"
#include "tvvf_vo_c/core/types.hpp"

namespace tvvf_vo_c {

/**
 * @brief マップから斥力用の障害物を抽出する際の設定
 */
struct MapRepulsionSettings {
    double max_distance = 0.0;          ///< ロボットからの最大距離[m]（0以下で無制限）
    double sampling_step = 0.5;         ///< サンプリングステップ[m]
    int8_t occupancy_threshold = 50;    ///< 占有セル判定の閾値
};

/**
 * @brief OccupancyGridから指定領域・距離内の占有セルを抽出する
 * @param map 占有グリッドマップ
 * @param region 対象領域（静的場生成と同一範囲を想定）
 * @param robot_pos ロボット位置（距離制限用）
 * @param settings 抽出設定
 * @return 障害物位置のリスト
 */
std::vector<Position> extract_map_obstacle_positions(
    const nav_msgs::msg::OccupancyGrid& map,
    const FieldRegion& region,
    const Position& robot_pos,
    const MapRepulsionSettings& settings);

/**
 * @brief 占有セルから MarkerArray を生成（可視化/公開用）
 * @param positions 障害物位置配列
 * @param scale 各マーカーのスケール（m）
 * @param frame_id Markerのフレーム
 * @param stamp タイムスタンプ
 * @return MarkerArray
 */
visualization_msgs::msg::MarkerArray create_map_obstacle_markers(
    const std::vector<Position>& positions,
    double scale,
    const std::string& frame_id,
    const builtin_interfaces::msg::Time& stamp);

}  // namespace tvvf_vo_c

#endif  // TVVF_VO_C_CORE_MAP_REPULSION_HPP_
