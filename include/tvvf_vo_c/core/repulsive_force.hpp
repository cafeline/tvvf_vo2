#ifndef TVVF_VO_C_CORE_REPULSIVE_FORCE_HPP_
#define TVVF_VO_C_CORE_REPULSIVE_FORCE_HPP_

#include <visualization_msgs/msg/marker_array.hpp>
#include <vector>
#include "tvvf_vo_c/core/types.hpp"

namespace tvvf_vo_c {

/**
 * @brief 2Dベクトル（斥力用）
 */
struct Vector2D {
    double x, y;
    
    Vector2D() : x(0.0), y(0.0) {}
    Vector2D(double x, double y) : x(x), y(y) {}
};

/**
 * @brief 斥力計算用の設定パラメータ
 */
struct RepulsiveForceConfig {
    double repulsive_strength = 1.0;  // 斥力の強度
    double influence_range = 2.0;     // 斥力の影響範囲[m]
};

/**
 * @brief 斥力計算クラス
 * 
 * 静的障害物からの斥力を計算し、ロボットの回避行動を生成する
 */
class RepulsiveForceCalculator {
public:
    /**
     * @brief コンストラクタ
     */
    RepulsiveForceCalculator() = default;
    
    /**
     * @brief 設定を更新
     * @param config 新しい設定
     */
    void setConfig(const RepulsiveForceConfig& config);
    
    /**
     * @brief 単一の障害物からの斥力を計算
     * @param robot_pos ロボットの現在位置
     * @param obstacle_pos 障害物の位置
     * @return 斥力ベクトル
     */
    Vector2D calculateForce(const Position& robot_pos, const Position& obstacle_pos) const;
    
    /**
     * @brief MarkerArrayから障害物位置を抽出
     * @param marker_array 障害物マーカー配列
     * @return 障害物位置のリスト
     */
    std::vector<Position> extractObstaclePositions(const visualization_msgs::msg::MarkerArray& marker_array) const;
    
    /**
     * @brief MarkerArrayから総斥力を計算
     * @param robot_pos ロボットの現在位置
     * @param marker_array 障害物マーカー配列
     * @return 総斥力ベクトル
     */
    Vector2D calculateTotalForce(const Position& robot_pos, 
                                 const visualization_msgs::msg::MarkerArray& marker_array) const;

    /**
     * @brief 事前に抽出済みの障害物位置から総斥力を計算
     * @param robot_pos ロボットの現在位置
     * @param obstacles 障害物位置配列
     * @return 総斥力ベクトル
     */
    Vector2D calculateTotalForceFromPositions(
        const Position& robot_pos,
        const std::vector<Position>& obstacles) const;
    
    /**
     * @brief 最大斥力を取得
     * @return 最大斥力の大きさ
     */
    double getMaximumForce() const;
    
private:
    RepulsiveForceConfig config_;
    
    /**
     * @brief 距離に基づく斥力の大きさを計算（線形減衰）
     * @param distance 障害物までの距離
     * @return 斥力の大きさ（スカラー値）
     */
    double calculateForceMagnitude(double distance) const;
};

} // namespace tvvf_vo_c

#endif // TVVF_VO_C_CORE_REPULSIVE_FORCE_HPP_
