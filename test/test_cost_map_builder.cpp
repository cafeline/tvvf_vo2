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

TEST_F(CostMapBuilderTest, EscapeSpeedRaisesZeroSpeedCellsNearRobot)
{
    settings.clearance_epsilon = 0.6;  // 近傍セルをゼロ速度にするため大きめに設定
    settings.min_speed = 0.05;
    settings.escape_speed_min = 0.2;
    builder = CostMapBuilder(settings);

    CostMapResult result;
    builder.build(map, result);
    ASSERT_TRUE(result.isValid());
    // 障害物の隣接セルは clearance <= epsilon なので速度0のまま
    EXPECT_DOUBLE_EQ(result.speedAt(2, 3), 0.0);

    // ロボット位置を隣接セル中心に置き、半径0.6mで特例適用
    Position robot_pos(1.25, 1.75);
    builder.apply_escape_speed(map, robot_pos, result, 0.6, settings.escape_speed_min);

    // 隣接セルは escape_speed_min で上書きされる
    EXPECT_DOUBLE_EQ(result.speedAt(2, 3), 0.2);
    // 障害物セル自体は上書きされない
    EXPECT_DOUBLE_EQ(result.speedAt(2, 2), 0.0);
}

TEST_F(CostMapBuilderTest, EscapeSpeedDoesNotAffectCellsOutsideRadius)
{
    settings.clearance_epsilon = 0.6;
    settings.min_speed = 0.05;
    settings.escape_speed_min = 0.1;
    builder = CostMapBuilder(settings);

    CostMapResult result;
    builder.build(map, result);
    ASSERT_TRUE(result.isValid());

    const double original_speed_far = result.speedAt(0, 0);
    Position robot_pos(1.25, 1.75);
    builder.apply_escape_speed(map, robot_pos, result, 0.4, settings.escape_speed_min);

    // 遠方セルは値が変化しない（escape_speed_min より大きいことを確認）
    EXPECT_DOUBLE_EQ(result.speedAt(0, 0), original_speed_far);
    EXPECT_GT(original_speed_far, settings.escape_speed_min);
}
