#include "tvvf_vo_c/core/repulsive_force.hpp"
#include <cmath>
#include <algorithm>
#include <limits>

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

std::vector<ObstacleHull> RepulsiveForceCalculator::extractObstacleHulls(
    const visualization_msgs::msg::MarkerArray& marker_array) const {

    std::vector<ObstacleHull> hulls;
    constexpr double DUP_TOL = 1e-9;

    auto is_duplicate = [&](const Position& a, const Position& b) {
        double dx = a.x - b.x;
        double dy = a.y - b.y;
        return (dx * dx + dy * dy) <= DUP_TOL;
    };

    for (const auto& marker : marker_array.markers) {
        if (marker.type != visualization_msgs::msg::Marker::LINE_LIST ||
            marker.action != visualization_msgs::msg::Marker::ADD ||
            marker.points.size() < 2) {
            continue;
        }

        std::vector<Position> vertices;
        vertices.reserve(marker.points.size() / 2 + 1);

        for (size_t i = 0; i + 1 < marker.points.size(); i += 2) {
            const auto& p = marker.points[i];
            Position v(p.x, p.y);
            if (vertices.empty() || !is_duplicate(vertices.back(), v)) {
                vertices.push_back(v);
            }
        }

        if (vertices.size() < 3) {
            continue;
        }

        // Centroid
        double sum_x = 0.0;
        double sum_y = 0.0;
        for (const auto& v : vertices) {
            sum_x += v.x;
            sum_y += v.y;
        }
        Position centroid(sum_x / static_cast<double>(vertices.size()),
                          sum_y / static_cast<double>(vertices.size()));

        hulls.push_back(ObstacleHull{vertices, centroid});
    }

    return hulls;
}

Vector2D RepulsiveForceCalculator::calculateForceToHull(
    const Position& robot_pos,
    const ObstacleHull& hull) const {

    if (hull.vertices.size() < 2) {
        return Vector2D(0.0, 0.0);
    }

    double min_dist_sq = std::numeric_limits<double>::max();
    Position nearest_point(0.0, 0.0);
    const size_t n = hull.vertices.size();

    for (size_t i = 0; i < n; ++i) {
        const Position& a = hull.vertices[i];
        const Position& b = hull.vertices[(i + 1) % n];

        const double abx = b.x - a.x;
        const double aby = b.y - a.y;
        const double len_sq = abx * abx + aby * aby;
        if (len_sq < 1e-12) {
            continue;
        }

        const double apx = robot_pos.x - a.x;
        const double apy = robot_pos.y - a.y;
        const double t = std::clamp((apx * abx + apy * aby) / len_sq, 0.0, 1.0);
        const double proj_x = a.x + t * abx;
        const double proj_y = a.y + t * aby;

        const double dx = robot_pos.x - proj_x;
        const double dy = robot_pos.y - proj_y;
        const double dist_sq = dx * dx + dy * dy;

        if (dist_sq < min_dist_sq) {
            min_dist_sq = dist_sq;
            nearest_point.x = proj_x;
            nearest_point.y = proj_y;
        }
    }

    if (min_dist_sq == std::numeric_limits<double>::max()) {
        return Vector2D(0.0, 0.0);
    }

    double distance = std::sqrt(min_dist_sq);
    if (distance < 1e-6) {
        distance = 1e-6;
    }
    if (distance >= config_.influence_range) {
        return Vector2D(0.0, 0.0);
    }

    const double force_magnitude = calculateForceMagnitude(distance);

    // force direction: from obstacle (nearest point) to robot
    const double dir_x = nearest_point.x - robot_pos.x;
    const double dir_y = nearest_point.y - robot_pos.y;
    const double dir_norm = std::sqrt(dir_x * dir_x + dir_y * dir_y);
    if (dir_norm < 1e-9) {
        return Vector2D(0.0, 0.0);
    }

    return Vector2D(
        -(dir_x / dir_norm) * force_magnitude,
        -(dir_y / dir_norm) * force_magnitude
    );
}

Vector2D RepulsiveForceCalculator::calculateTotalForceFromHulls(
    const Position& robot_pos,
    const std::vector<ObstacleHull>& obstacles) const {

    Vector2D total_force(0.0, 0.0);
    for (const auto& hull : obstacles) {
        auto f = calculateForceToHull(robot_pos, hull);
        total_force.x += f.x;
        total_force.y += f.y;
    }
    return total_force;
}

bool RepulsiveForceCalculator::isInsideConvexPolygon(
    const Position& p,
    const std::vector<Position>& vertices) const {
    const size_t n = vertices.size();
    if (n < 3) {
        return false;
    }
    auto cross = [](const Position& o, const Position& a, const Position& b) {
        return (a.x - o.x) * (b.y - o.y) - (a.y - o.y) * (b.x - o.x);
    };

    bool has_pos = false, has_neg = false;
    for (size_t i = 0; i < n; ++i) {
        const Position& v1 = vertices[i];
        const Position& v2 = vertices[(i + 1) % n];
        const double c = cross(v1, v2, p);
        has_pos = has_pos || (c > 1e-9);
        has_neg = has_neg || (c < -1e-9);
        if (has_pos && has_neg) {
            return false;
        }
    }
    return true;
}

} // namespace tvvf_vo_c
