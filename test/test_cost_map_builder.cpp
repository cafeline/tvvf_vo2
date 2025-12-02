// test/test_cost_map_builder.cpp

#include <gtest/gtest.h>
#include "tvvf_vo_c/core/cost_map_builder.hpp"
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <cmath>

using namespace tvvf_vo_c;

class CostMapBuilderTest : public ::testing::Test {
protected:
    CostMapSettings settings;
    CostMapBuilder builder;
    nav_msgs::msg::OccupancyGrid map;

    CostMapBuilderTest()
        : settings(), builder(settings) {}

    void SetUp() override {
        map.info.width = 5;
        map.info.height = 5;
        map.info.resolution = 0.5;
        map.info.origin.position.x = 0.0;
        map.info.origin.position.y = 0.0;
        map.data.assign(map.info.width * map.info.height, 0);
        map.data[2 * map.info.width + 2] = 100;
    }
};

TEST_F(CostMapBuilderTest, ComputesClearanceAroundObstacle) {
    auto result = builder.build(map);
    ASSERT_TRUE(result.isValid());

    EXPECT_DOUBLE_EQ(result.clearanceAt(2, 2), 0.0);
    EXPECT_NEAR(result.clearanceAt(2, 3), 0.5, 1e-3);
    EXPECT_NEAR(result.clearanceAt(1, 1), std::sqrt(2) * 0.5, 5e-2);
}

TEST_F(CostMapBuilderTest, AdjustsSpeedBasedOnDistance) {
    auto result = builder.build(map);
    ASSERT_TRUE(result.isValid());

    const double speed_near = result.speedAt(2, 3);
    const double speed_far = result.speedAt(0, 0);
    EXPECT_LT(speed_near, speed_far);
    EXPECT_DOUBLE_EQ(result.speedAt(2, 2), 0.0);
}

TEST_F(CostMapBuilderTest, ReusesAllocatedBuffersWhenSizeIsStable) {
    CostMapResult reusable;
    builder.build(map, reusable);
    auto* speed_ptr = reusable.speed_layer.data();
    auto* clearance_ptr = reusable.clearance_layer.data();

    builder.build(map, reusable);

    EXPECT_EQ(speed_ptr, reusable.speed_layer.data());
    EXPECT_EQ(clearance_ptr, reusable.clearance_layer.data());
    ASSERT_TRUE(reusable.isValid());
}
