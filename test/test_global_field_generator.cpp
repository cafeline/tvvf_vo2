// test/test_global_field_generator.cpp

#include <gtest/gtest.h>
#include "tvvf_vo_c/core/global_field_generator.hpp"
#include "tvvf_vo_c/core/region_utils.hpp"
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <chrono>
#include <cmath>

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
    auto field = generator.generateField({});
    EXPECT_EQ(field.width, 20);
    EXPECT_EQ(field.height, 20);
}

TEST_F(GlobalFieldGeneratorTest, CropsStaticFieldToRegion) {
    Position goal(4.0, 4.0);
    FieldRegion region(Position(2.0, 3.0), Position(6.0, 7.0));

    generator.precomputeStaticField(test_map, goal, region);
    auto field = generator.generateField({});

    EXPECT_EQ(field.width, 8);   // (6-2)/0.5 = 8 cells
    EXPECT_EQ(field.height, 8);  // (7-3)/0.5 = 8 cells
    EXPECT_NEAR(field.origin.x, 2.0, 1e-6);
    EXPECT_NEAR(field.origin.y, 3.0, 1e-6);
    ASSERT_TRUE(field.region.has_value());
    EXPECT_NEAR(field.region->min.x, 2.0, 1e-6);
    EXPECT_NEAR(field.region->max.x, 6.0, 1e-6);
    EXPECT_TRUE(field.region->contains(Position(4.0, 4.0)));
}

// TEST 2: 動的障害物なしの場合
TEST_F(GlobalFieldGeneratorTest, NoDynamicObstacles) {
    // Given: 静的場が計算済み、動的障害物なし
    Position goal(8.0, 8.0);
    generator.precomputeStaticField(test_map, goal);
    std::vector<DynamicObstacle> obstacles;
    
    // When: 動的障害物を含むフィールドを生成
    auto field = generator.generateField(obstacles);
    
    // Then: フィールドが生成されていること（空の実装なので常に失敗する）
    // Red Phase: このテストは失敗することが期待される
    EXPECT_NE(field.width, 0) << "Field should be generated";
    EXPECT_NE(field.height, 0) << "Field should be generated";
}

// TEST 3: 単一動的障害物の斥力テスト
TEST_F(GlobalFieldGeneratorTest, SingleDynamicObstacle) {
    // Given: 静的場と1つの動的障害物
    Position goal(8.0, 8.0);
    generator.precomputeStaticField(test_map, goal);
    
    // 障害物: 位置(5,4), 速度(0,0), 半径0.5m
    DynamicObstacle obstacle(Position(5.0, 4.0), Velocity(0, 0), 0.5);
    std::vector<DynamicObstacle> obstacles = {obstacle};
    
    // When: フィールドを生成
    auto field = generator.generateField(obstacles);
    
    // Then: フィールドが生成されていること（空の実装なので常に失敗する）
    // Red Phase: このテストは失敗することが期待される
    EXPECT_NE(field.width, 0) << "Field should handle dynamic obstacles";
    EXPECT_NE(field.height, 0) << "Field should handle dynamic obstacles";
}

// TEST 4: 複数動的障害物の統合テスト
TEST_F(GlobalFieldGeneratorTest, MultipleDynamicObstacles) {
    // Given: 複数の動的障害物
    Position goal(10.0, 10.0);
    generator.precomputeStaticField(test_map, goal);
    
    std::vector<DynamicObstacle> obstacles = {
        DynamicObstacle(Position(6.0, 5.0), Velocity(0, 0), 0.5),
        DynamicObstacle(Position(5.0, 6.0), Velocity(0, 0), 0.5)
    };
    
    // When: 動的ベクトルを計算
    auto field = generator.generateField(obstacles);
    
    // Then: フィールドが適切に生成される（空の実装なので失敗する）
    // Red Phase: このテストは失敗することが期待される
    EXPECT_NE(field.width, 0) << "Field should handle multiple obstacles";
    EXPECT_NE(field.height, 0) << "Field should handle multiple obstacles";
}

// TEST 5: リアルタイム性能テスト
TEST_F(GlobalFieldGeneratorTest, RealtimePerformance) {
    // Given: 事前計算済みの静的場と多数の動的障害物
    Position goal(10.0, 10.0);
    generator.precomputeStaticField(test_map, goal);
    
    std::vector<DynamicObstacle> obstacles;
    for (int i = 0; i < 10; i++) {
        obstacles.push_back(
            DynamicObstacle(
                Position(i * 1.0, i * 1.0), 
                Velocity(0.1, 0.1), 
                0.3)
        );
    }
    
    // When: 100回フィールド生成を実行
    auto start = std::chrono::high_resolution_clock::now();
    for (int i = 0; i < 100; i++) {
        generator.generateField(obstacles);
    }
    auto end = std::chrono::high_resolution_clock::now();
    
    // Then: 平均計算時間が10ms以下
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
    double avg_time_ms = duration.count() / 100.0 / 1000.0;
    // Red Phase: 空の実装なので非常に高速に実行されるため、このテストはパスする可能性がある
    EXPECT_LT(avg_time_ms, 10.0);
}

// TEST 6: ブレンディングのテスト
TEST_F(GlobalFieldGeneratorTest, BlendingWithDynamicObstacles) {
    // Given: 静的場が計算済み
    Position goal(10.0, 10.0);
    generator.precomputeStaticField(test_map, goal);
    
    // 空のフィールドを作成（テスト用）
    VectorField static_field;
    static_field.width = 20;
    static_field.height = 20;
    static_field.resolution = 0.5;
    static_field.origin = Position(0, 0);
    
    // グリッドとベクトルを初期化
    static_field.grid.resize(20);
    static_field.vectors.resize(20);
    for (int i = 0; i < 20; i++) {
        static_field.grid[i].resize(20);
        static_field.vectors[i].resize(20, {0.1, 0.1});  // 小さな初期ベクトル
    }
    
    // 動的障害物を設定
    std::vector<DynamicObstacle> obstacles = {
        DynamicObstacle(Position(5.0, 5.0), Velocity(0, 0), 0.5)
    };
    
    // When: ブレンディングを実行
    auto blended_field = generator.blendWithDynamicObstacles(static_field, obstacles);
    
    // Then: ブレンドされたフィールドが生成される（空の実装なので失敗する）
    // Red Phase: このテストは失敗することが期待される
    EXPECT_NE(blended_field.width, 0) << "Blended field should be generated";
    EXPECT_NE(blended_field.height, 0) << "Blended field should be generated";
}

// メイン関数
int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
