// test/test_global_field_generator.cpp

#include <gtest/gtest.h>
#include "tvvf_vo_c/core/global_field_generator.hpp"
#include "tvvf_vo_c/core/region_utils.hpp"
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <chrono>
#include <cmath>
#include <algorithm>

using namespace tvvf_vo_c;

TEST(PlanningRegionTest, IncludesMarginAndWidth) {
    Position start(0.0, 0.0);
    Position goal(10.0, -2.0);
    const double path_width = 6.0;
    const double margin = 5.0;

    auto region = create_path_region(start, goal, path_width, margin);

    EXPECT_NEAR(region.min.x, -8.0, 1e-6);   // 0 - (3 + 5)
    EXPECT_NEAR(region.max.x, 18.0, 1e-6);   // 10 + (3 + 5)
    EXPECT_NEAR(region.min.y, -10.0, 1e-6);  // -2 - (3 + 5)
    EXPECT_NEAR(region.max.y, 8.0, 1e-6);    // 0 + (3 + 5)
    EXPECT_TRUE(region.contains(start));
    EXPECT_TRUE(region.contains(goal));
}
class GlobalFieldGeneratorTest : public ::testing::Test {
protected:
    GlobalFieldGenerator generator;
    nav_msgs::msg::OccupancyGrid test_map;
    
    void SetUp() override {
        test_map.info.width = 20;
        test_map.info.height = 20;
        test_map.info.resolution = 0.5;
        test_map.info.origin.position.x = 0.0;
        test_map.info.origin.position.y = 0.0;
        test_map.data.resize(400, 0);
    }
};

// TEST 1: 静的場の事前計算テスト
TEST_F(GlobalFieldGeneratorTest, PrecomputeStaticField) {
    // Given: マップとゴール
    Position goal(8.0, 8.0);
    
    // When: 静的場を事前計算
    generator.precomputeStaticField(test_map, goal);
    
    // Then: 静的場が計算されていること
    // フィールドが生成されて、サイズが正しいことを確認
    auto field = generator.computeFieldOnTheFly(test_map, goal, std::nullopt);
    EXPECT_EQ(field.width, 20);
    EXPECT_EQ(field.height, 20);
}

TEST_F(GlobalFieldGeneratorTest, StoresCostMapResult) {
    Position goal(8.0, 8.0);
    generator.precomputeStaticField(test_map, goal);

    const auto& cost_map = generator.getLastCostMap();
    ASSERT_TRUE(cost_map.has_value());
    EXPECT_EQ(cost_map->width, test_map.info.width);
    EXPECT_EQ(cost_map->height, test_map.info.height);
}

TEST_F(GlobalFieldGeneratorTest, CropsStaticFieldToRegion) {
    Position goal(4.0, 4.0);
    FieldRegion region(Position(2.0, 3.0), Position(6.0, 7.0));

    generator.precomputeStaticField(test_map, goal, region);
    auto field = generator.computeFieldOnTheFly(test_map, goal, region);

    EXPECT_EQ(field.width, 8);   // (6-2)/0.5 = 8 cells
    EXPECT_EQ(field.height, 8);  // (7-3)/0.5 = 8 cells
    EXPECT_NEAR(field.origin.x, 2.0, 1e-6);
    EXPECT_NEAR(field.origin.y, 3.0, 1e-6);
    ASSERT_TRUE(field.region.has_value());
    EXPECT_NEAR(field.region->min.x, 2.0, 1e-6);
    EXPECT_NEAR(field.region->max.x, 6.0, 1e-6);
    EXPECT_TRUE(field.region->contains(Position(4.0, 4.0)));
}

TEST_F(GlobalFieldGeneratorTest, EndToEndGradientGuidanceAvoidsWall) {
    test_map.data.assign(test_map.data.size(), 0);

    Position goal(8.0, 1.0);
    generator.precomputeStaticField(test_map, goal);

    Position pose(1.0, 1.0);
    bool reached = false;
    const double max_x = test_map.info.width * test_map.info.resolution;
    const double max_y = test_map.info.height * test_map.info.resolution;

    const double step_size = 0.05;
    for (int step = 0; step < 400; ++step) {
        auto vec = generator.getVelocityAt(pose);
        const double norm = std::sqrt(vec[0] * vec[0] + vec[1] * vec[1]);
        if (norm < 1e-4) {
            break;
        }

        pose.x = std::clamp(pose.x + vec[0] * step_size, 0.0, max_x);
        pose.y = std::clamp(pose.y + vec[1] * step_size, 0.0, max_y);

        if (std::hypot(goal.x - pose.x, goal.y - pose.y) < 0.3) {
            reached = true;
            break;
        }
    }

    EXPECT_TRUE(reached);
    if (!reached) {
        ADD_FAILURE() << "Final pose (" << pose.x << ", " << pose.y << ")";
    }
}

TEST_F(GlobalFieldGeneratorTest, ComputesFieldOnTheFlyFromOccupancyGrid) {
    // マップは10x10m、解像度1.0mの自由空間
    nav_msgs::msg::OccupancyGrid map;
    map.info.width = 10;
    map.info.height = 10;
    map.info.resolution = 1.0;
    map.info.origin.position.x = 0.0;
    map.info.origin.position.y = 0.0;
    map.data.assign(map.info.width * map.info.height, 0);

    // ゴールを設定（事前計算なし）
    Position goal(9.0, 5.0);

    // On-the-flyでベクトル場を生成（OccupancyGridのみ）
    auto field = generator.computeFieldOnTheFly(map, goal, std::nullopt);
    ASSERT_GT(field.width, 0);
    ASSERT_GT(field.height, 0);

    // 生成されたベクトル場でスタート(1,5)からゴールへ進むシミュレーション
    Position pose(1.0, 5.0);
    const double step_size = 0.1;
    for (int i = 0; i < 400; ++i) {
        auto vec = field.getVector(pose);
        const double norm = std::hypot(vec[0], vec[1]);
        if (norm < 1e-4) {
            break;
        }

        pose.x += vec[0] / norm * step_size;
        pose.y += vec[1] / norm * step_size;

        const double dist_goal = std::hypot(goal.x - pose.x, goal.y - pose.y);
        if (dist_goal < 0.5) {
            break;
        }
    }

    EXPECT_LT(std::hypot(goal.x - pose.x, goal.y - pose.y), 1.5)
        << "final pose=(" << pose.x << ", " << pose.y << ")";
}

// メイン関数
int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
