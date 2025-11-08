#include "tvvf_vo_c/core/repulsive_force.hpp"
#include <cmath>
#include <algorithm>

namespace tvvf_vo_c {

void RepulsiveForceCalculator::setConfig(const RepulsiveForceConfig& config) {
    config_ = config;
}

Vector2D RepulsiveForceCalculator::calculateForce(
    const Position& robot_pos, const Position& obstacle_pos) const {
    
    // 障害物までの距離を計算
    double dx = obstacle_pos.x - robot_pos.x;
    double dy = obstacle_pos.y - robot_pos.y;
    double distance = std::sqrt(dx * dx + dy * dy);
    
    // 影響範囲外の場合は斥力ゼロ
    if (distance >= config_.influence_range) {
        return Vector2D(0.0, 0.0);
    }
    
    // 距離が非常に小さい場合の処理（ゼロ除算防止）
    if (distance < 0.01) {
        distance = 0.01;
        // ランダムな方向に最大斥力を適用
        double angle = 0.0;  // 簡単のため固定方向
        return Vector2D(
            -config_.repulsive_strength * std::cos(angle),
            -config_.repulsive_strength * std::sin(angle)
        );
    }
    
    // 斥力の大きさを計算（線形減衰）
    double force_magnitude = calculateForceMagnitude(distance);
    
    // 斥力の方向（障害物から離れる方向）
    double force_x = -(dx / distance) * force_magnitude;
    double force_y = -(dy / distance) * force_magnitude;
    
    return Vector2D(force_x, force_y);
}

std::vector<Position> RepulsiveForceCalculator::extractObstaclePositions(
    const visualization_msgs::msg::MarkerArray& marker_array) const {
    
    std::vector<Position> obstacles;
    
    for (const auto& marker : marker_array.markers) {
        // マーカーの位置を障害物位置として抽出
        Position obs_pos(marker.pose.position.x, marker.pose.position.y);
        obstacles.push_back(obs_pos);
    }
    
    return obstacles;
}

Vector2D RepulsiveForceCalculator::calculateTotalForce(
    const Position& robot_pos,
    const visualization_msgs::msg::MarkerArray& marker_array) const {
    
    // 障害物位置を抽出
    auto obstacles = extractObstaclePositions(marker_array);
    
    return calculateTotalForceFromPositions(robot_pos, obstacles);
}

Vector2D RepulsiveForceCalculator::calculateTotalForceFromPositions(
    const Position& robot_pos,
    const std::vector<Position>& obstacles) const {

    // 全障害物からの斥力を合成
    Vector2D total_force(0.0, 0.0);
    
    for (const auto& obstacle : obstacles) {
        auto force = calculateForce(robot_pos, obstacle);
        total_force.x += force.x;
        total_force.y += force.y;
    }
    
    return total_force;
}

double RepulsiveForceCalculator::getMaximumForce() const {
    return config_.repulsive_strength;
}

double RepulsiveForceCalculator::calculateForceMagnitude(double distance) const {
    // 影響範囲内で線形に減衰する斥力
    // distance = 0 で最大、distance = influence_range で 0
    if (distance >= config_.influence_range) {
        return 0.0;
    }
    
    // 線形減衰: (influence_range - distance) / influence_range * strength
    double normalized_distance = (config_.influence_range - distance) / config_.influence_range;
    return normalized_distance * config_.repulsive_strength;
}

} // namespace tvvf_vo_c
