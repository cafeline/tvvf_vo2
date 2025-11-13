#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>

#include "tvvf_vo_c/core/map_repulsion.hpp"

#include <builtin_interfaces/msg/time.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

using namespace tvvf_vo_c;

namespace
{
nav_msgs::msg::OccupancyGrid make_test_map()
{
    nav_msgs::msg::OccupancyGrid map;
    map.info.width = 5;
    map.info.height = 5;
    map.info.resolution = 1.0;
    map.info.origin.position.x = 0.0;
    map.info.origin.position.y = 0.0;
    map.data.assign(map.info.width * map.info.height, 0);
    return map;
}

void set_occupancy(nav_msgs::msg::OccupancyGrid& map, int x, int y, int8_t value)
{
    const int index = y * static_cast<int>(map.info.width) + x;
    map.data[index] = value;
}
}  // namespace

TEST(MapRepulsionExtractionTest, FiltersByRegionAndDistance)
{
    auto map = make_test_map();
    set_occupancy(map, 1, 1, 100);  // within region/distance
    set_occupancy(map, 4, 4, 100);  // outside region
    set_occupancy(map, 2, 0, 100);  // inside region but near limit

    FieldRegion region(Position(0.0, 0.0), Position(3.0, 3.0));
    MapRepulsionSettings settings;
    settings.max_distance = 3.0;
    settings.sampling_step = 1.0;
    settings.occupancy_threshold = 50;

    const Position robot_pos(0.0, 0.0);
    const auto obstacles = extract_map_obstacle_positions(map, region, robot_pos, settings);

    ASSERT_EQ(obstacles.size(), 2u);
    auto contains_position = [&](double x, double y) {
        return std::any_of(
            obstacles.begin(), obstacles.end(),
            [&](const Position& pos) {
                return std::fabs(pos.x - x) < 1e-6 &&
                       std::fabs(pos.y - y) < 1e-6;
            });
    };
    EXPECT_TRUE(contains_position(1.5, 1.5));
    EXPECT_TRUE(contains_position(2.5, 0.5));
}

TEST(MapRepulsionExtractionTest, HonorsSamplingStep)
{
    auto map = make_test_map();
    set_occupancy(map, 1, 2, 100);
    set_occupancy(map, 2, 2, 100);

    FieldRegion region(Position(0.0, 0.0), Position(5.0, 5.0));
    MapRepulsionSettings settings;
    settings.max_distance = 10.0;
    settings.sampling_step = 2.0;  // stride of 2 cells
    settings.occupancy_threshold = 50;

    const Position robot_pos(0.0, 0.0);
    const auto obstacles = extract_map_obstacle_positions(map, region, robot_pos, settings);

    ASSERT_EQ(obstacles.size(), 1u);
    EXPECT_NEAR(obstacles[0].x, 2.5, 1e-6);  // cell (2,2) survives
    EXPECT_NEAR(obstacles[0].y, 2.5, 1e-6);
}

TEST(MapRepulsionExtractionTest, ReturnsEmptyWhenRegionInvalid)
{
    auto map = make_test_map();
    set_occupancy(map, 0, 0, 100);

    FieldRegion invalid_region(Position(3.0, 3.0), Position(1.0, 1.0));
    MapRepulsionSettings settings;
    settings.max_distance = 10.0;

    const Position robot_pos(0.0, 0.0);
    const auto obstacles = extract_map_obstacle_positions(map, invalid_region, robot_pos, settings);

    EXPECT_TRUE(obstacles.empty());
}

TEST(MapRepulsionMarkersTest, CreatesMarkersWithConsistentAttributes)
{
    std::vector<Position> positions = {
        Position(1.0, 2.0),
        Position(-0.5, 3.5)
    };

    const double scale = 0.4;
    const std::string frame_id = "map";

    builtin_interfaces::msg::Time stamp;
    stamp.sec = 123;
    stamp.nanosec = 456u;

    const auto markers = create_map_obstacle_markers(
        positions, scale, frame_id, stamp);

    ASSERT_EQ(markers.markers.size(), positions.size());
    for (size_t i = 0; i < positions.size(); ++i) {
        const auto& marker = markers.markers[i];
        EXPECT_EQ(marker.header.frame_id, frame_id);
        EXPECT_EQ(marker.header.stamp.sec, stamp.sec);
        EXPECT_EQ(marker.ns, "map_static_obstacles");
        EXPECT_EQ(marker.id, static_cast<int>(i));
        EXPECT_DOUBLE_EQ(marker.pose.position.x, positions[i].x);
        EXPECT_DOUBLE_EQ(marker.pose.position.y, positions[i].y);
        EXPECT_NEAR(marker.scale.x, scale, 1e-9);
        EXPECT_NEAR(marker.scale.y, scale, 1e-9);
        EXPECT_NEAR(marker.scale.z, 0.1, 1e-9);
        EXPECT_NEAR(marker.color.a, 0.6, 1e-6);
    }
}
