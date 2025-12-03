#include <gtest/gtest.h>
#include "tvvf_vo_c/core/field_types.hpp"
#include "tvvf_vo_c/core/global_field_generator.hpp"
#include "tvvf_vo_c/core/cost_map_builder.hpp"
#include <nav_msgs/msg/occupancy_grid.hpp>

using namespace tvvf_vo_c;

TEST(VectorFieldOverlayTest, UsesFinestOverlayInsideRegion) {
    VectorField base;
    base.resize(2, 2);
    base.resolution = 0.2;
    base.origin = Position(0.0, 0.0);
    for (int y = 0; y < base.height; ++y) {
        for (int x = 0; x < base.width; ++x) {
            base.vectors[static_cast<size_t>(y)][static_cast<size_t>(x)] = {1.0, 0.0};
        }
    }

    VectorField overlay;
    overlay.resize(2, 2);
    overlay.resolution = 0.1;
    overlay.origin = Position(0.0, 0.0);
    for (int y = 0; y < overlay.height; ++y) {
        for (int x = 0; x < overlay.width; ++x) {
            overlay.vectors[static_cast<size_t>(y)][static_cast<size_t>(x)] = {0.0, 1.0};
        }
    }

    base.addOverlay(overlay);

    auto vec_inside = base.getVector(Position(0.05, 0.05));
    EXPECT_NEAR(vec_inside[0], 0.0, 1e-6);
    EXPECT_NEAR(vec_inside[1], 1.0, 1e-6);

    auto vec_outside = base.getVector(Position(0.25, 0.25));
    EXPECT_NEAR(vec_outside[0], 1.0, 1e-6);
    EXPECT_NEAR(vec_outside[1], 0.0, 1e-6);
}

TEST(CostMapOverlayTest, SpeedAtWorldPrefersOverlay) {
    CostMapResult base;
    base.width = 2;
    base.height = 2;
    base.resolution = 0.2;
    base.origin = Position(0.0, 0.0);
    base.speed_layer.assign(base.width * base.height, 1.0);
    base.clearance_layer.assign(base.width * base.height, 1.0);

    CostMapResult overlay;
    overlay.width = 2;
    overlay.height = 2;
    overlay.resolution = 0.1;
    overlay.origin = Position(0.0, 0.0);
    overlay.speed_layer.assign(overlay.width * overlay.height, 0.5);
    overlay.clearance_layer.assign(overlay.width * overlay.height, 0.2);

    base.overlays.push_back(overlay.toLayer());

    const double speed_overlay = base.speedAtWorld(Position(0.05, 0.05));
    EXPECT_NEAR(speed_overlay, 0.5, 1e-6);

    const double speed_base = base.speedAtWorld(Position(0.25, 0.25));
    EXPECT_NEAR(speed_base, 1.0, 1e-6);

    const double clearance_overlay = base.clearanceAtWorld(Position(0.05, 0.05));
    EXPECT_NEAR(clearance_overlay, 0.2, 1e-6);
}

TEST(GlobalFieldGeneratorMultiResTest, BuildsLocalOverlay) {
    nav_msgs::msg::OccupancyGrid map;
    map.info.width = 20;
    map.info.height = 20;
    map.info.resolution = 0.2;
    map.info.origin.position.x = 0.0;
    map.info.origin.position.y = 0.0;
    map.data.assign(map.info.width * map.info.height, 0);

    Position goal(2.0, 2.0);
    Position focus(1.0, 1.0);

    GlobalFieldGenerator generator;
    GlobalFieldGenerator::MultiResolutionSettings settings;
    settings.global_resolution = 0.2;
    settings.local_resolution = 0.1;
    settings.local_radius = 1.0;
    generator.setMultiResolutionSettings(settings);

    auto field = generator.computeFieldOnTheFly(map, goal, std::nullopt, focus);
    auto cost_map = generator.getLastCostMap();
    ASSERT_TRUE(cost_map.has_value());

    ASSERT_FALSE(field.overlays.empty());
    EXPECT_NEAR(field.overlays.front().resolution, 0.1, 1e-6);

    ASSERT_FALSE(cost_map->overlays.empty());
    EXPECT_NEAR(cost_map->overlays.front().resolution, 0.1, 1e-6);
    EXPECT_GT(cost_map->overlays.front().width, 0u);
    EXPECT_GT(cost_map->overlays.front().height, 0u);
}

TEST(GlobalFieldGeneratorMultiResTest, BuildsLocalCostOverlayEvenIfGoalOutside) {
    nav_msgs::msg::OccupancyGrid map;
    map.info.width = 20;
    map.info.height = 20;
    map.info.resolution = 0.2;
    map.info.origin.position.x = 0.0;
    map.info.origin.position.y = 0.0;
    map.data.assign(map.info.width * map.info.height, 0);

    Position goal_far(5.0, 5.0);
    Position focus(0.0, 0.0);

    GlobalFieldGenerator generator;
    GlobalFieldGenerator::MultiResolutionSettings settings;
    settings.global_resolution = 0.2;
    settings.local_resolution = 0.1;
    settings.local_radius = 1.0;
    generator.setMultiResolutionSettings(settings);

    auto field = generator.computeFieldOnTheFly(map, goal_far, std::nullopt, focus);
    auto cost_map = generator.getLastCostMap();
    ASSERT_TRUE(cost_map.has_value());

    EXPECT_FALSE(cost_map->overlays.empty());
    // ゴールが外でもベクトルオーバーレイが生成される
    EXPECT_FALSE(field.overlays.empty());
}

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
