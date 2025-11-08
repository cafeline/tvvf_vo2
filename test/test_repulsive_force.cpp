#include <gtest/gtest.h>

#include "tvvf_vo_c/core/repulsive_force.hpp"
#include "tvvf_vo_c/core/types.hpp"

#include <visualization_msgs/msg/marker_array.hpp>

using namespace tvvf_vo_c;

namespace
{
visualization_msgs::msg::Marker make_marker(double x, double y)
{
    visualization_msgs::msg::Marker marker;
    marker.pose.position.x = x;
    marker.pose.position.y = y;
    marker.pose.position.z = 0.0;
    marker.type = visualization_msgs::msg::Marker::SPHERE;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.scale.x = marker.scale.y = marker.scale.z = 1.0;
    marker.color.a = 1.0;
    return marker;
}
}  // namespace

TEST(RepulsiveForceCalculatorCacheTest, MatchesMarkerArrayComputation)
{
    RepulsiveForceCalculator calculator;
    RepulsiveForceConfig config;
    config.repulsive_strength = 2.0;
    config.influence_range = 5.0;
    calculator.setConfig(config);

    visualization_msgs::msg::MarkerArray markers;
    markers.markers.push_back(make_marker(1.0, 0.0));
    markers.markers.push_back(make_marker(3.0, 4.0));

    const Position robot_pos(0.5, 0.0);

    const auto marker_force = calculator.calculateTotalForce(robot_pos, markers);
    const auto cached_positions = calculator.extractObstaclePositions(markers);
    const auto cached_force = calculator.calculateTotalForceFromPositions(robot_pos, cached_positions);

    EXPECT_NEAR(marker_force.x, cached_force.x, 1e-9);
    EXPECT_NEAR(marker_force.y, cached_force.y, 1e-9);
}

TEST(RepulsiveForceCalculatorCacheTest, ZeroForceWhenCacheIsEmpty)
{
    RepulsiveForceCalculator calculator;
    RepulsiveForceConfig config;
    config.repulsive_strength = 1.0;
    config.influence_range = 2.0;
    calculator.setConfig(config);

    const Position robot_pos(0.0, 0.0);
    const std::vector<Position> empty_cache;

    const auto force = calculator.calculateTotalForceFromPositions(robot_pos, empty_cache);

    EXPECT_DOUBLE_EQ(force.x, 0.0);
    EXPECT_DOUBLE_EQ(force.y, 0.0);
}
