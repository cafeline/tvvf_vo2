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

visualization_msgs::msg::Marker make_square_hull_marker()
{
    // Convex hull as LINE_LIST (square with corners +/-0.5)
    visualization_msgs::msg::Marker marker;
    marker.type = visualization_msgs::msg::Marker::LINE_LIST;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.header.frame_id = "map";
    marker.scale.x = 0.05;
    marker.color.a = 1.0;

    std::vector<std::pair<double, double>> corners = {
        {-0.5, -0.5}, {0.5, -0.5}, {0.5, 0.5}, {-0.5, 0.5}
    };
    for (size_t i = 0; i < corners.size(); ++i) {
        auto a = corners[i];
        auto b = corners[(i + 1) % corners.size()];
        geometry_msgs::msg::Point p1, p2;
        p1.x = a.first; p1.y = a.second; p1.z = 0.0;
        p2.x = b.first; p2.y = b.second; p2.z = 0.0;
        marker.points.push_back(p1);
        marker.points.push_back(p2);
    }
    return marker;
}

TEST(RepulsiveForceCalculatorHullTest, ExtractsConvexHullFromLineList)
{
    RepulsiveForceCalculator calculator;
    visualization_msgs::msg::MarkerArray markers;
    markers.markers.push_back(make_square_hull_marker());

    auto hulls = calculator.extractObstacleHulls(markers);
    ASSERT_EQ(hulls.size(), 1u);
    EXPECT_EQ(hulls[0].vertices.size(), 4u);
    EXPECT_NEAR(hulls[0].centroid.x, 0.0, 1e-9);
    EXPECT_NEAR(hulls[0].centroid.y, 0.0, 1e-9);
}

TEST(RepulsiveForceCalculatorHullTest, ForceUsesNearestPointOnHull)
{
    RepulsiveForceCalculator calculator;
    RepulsiveForceConfig config;
    config.repulsive_strength = 2.0;
    config.influence_range = 5.0;
    calculator.setConfig(config);

    visualization_msgs::msg::MarkerArray markers;
    markers.markers.push_back(make_square_hull_marker());

    auto hulls = calculator.extractObstacleHulls(markers);
    ASSERT_EQ(hulls.size(), 1u);

    // Robot is to the +x side; nearest hull point is (0.5, 0)
    const Position robot_pos(2.0, 0.0);
    const auto force = calculator.calculateTotalForceFromHulls(robot_pos, hulls);

    // Distance to hull surface = 1.5 -> linear decay: (5 - 1.5) / 5 * 2 = 1.4
    EXPECT_NEAR(force.x, 1.4, 1e-3);
    EXPECT_NEAR(force.y, 0.0, 1e-6);
}
