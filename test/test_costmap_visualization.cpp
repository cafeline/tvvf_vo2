// test/test_costmap_visualization.cpp

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>

#include "tvvf_vo_c/core/cost_map_builder.hpp"
#include "tvvf_vo_c/ros/tvvf_vo_node.hpp"

namespace
{
class CostmapVisualizationTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    int argc = 0;
    rclcpp::init(argc, nullptr);
  }

  static void TearDownTestSuite()
  {
    rclcpp::shutdown();
  }
};

nav_msgs::msg::OccupancyGrid makeCenteredObstacleMap(std::uint32_t size)
{
  nav_msgs::msg::OccupancyGrid map;
  map.header.frame_id = "map";
  map.info.resolution = 1.0;
  map.info.width = size;
  map.info.height = size;
  map.info.origin.position.x = 0.0;
  map.info.origin.position.y = 0.0;
  map.info.origin.orientation.w = 1.0;
  map.data.assign(size * size, 0);

  const std::uint32_t center = size / 2;
  const std::size_t idx = static_cast<std::size_t>(center) * size + center;
  map.data[idx] = 100;
  return map;
}
}  // namespace

TEST_F(CostmapVisualizationTest, ConvertsClearanceToOccupancyCosts)
{
  auto node = std::make_shared<tvvf_vo_c::TVVFVONode>();

  tvvf_vo_c::CostMapSettings settings;
  settings.free_threshold = 10.0;
  settings.occupied_threshold = 50.0;
  settings.alpha = 1.0;
  settings.min_speed = 0.05;
  settings.max_speed = 1.0;
  settings.clearance_epsilon = 0.1;
  settings.max_clearance = 5.0;

  tvvf_vo_c::CostMapBuilder builder(settings);
  auto map = makeCenteredObstacleMap(11);
  auto cost_result = builder.build(map);
  ASSERT_TRUE(cost_result.isValid());

  tvvf_vo_c::VectorField dummy_field;
  dummy_field.width = static_cast<int>(map.info.width);
  dummy_field.height = static_cast<int>(map.info.height);
  dummy_field.resolution = map.info.resolution;
  dummy_field.origin = tvvf_vo_c::Position(
      map.info.origin.position.x, map.info.origin.position.y);

  auto grid = node->debug_build_costmap_grid(cost_result, dummy_field, map.header.frame_id);

  ASSERT_EQ(grid.info.width, map.info.width);
  ASSERT_EQ(grid.info.height, map.info.height);
  ASSERT_EQ(grid.data.size(), grid.info.width * grid.info.height);

  auto idx = [&](std::uint32_t x, std::uint32_t y) -> int {
    return grid.data[static_cast<std::size_t>(y) * grid.info.width + x];
  };

  const std::uint32_t center = map.info.width / 2;
  EXPECT_EQ(idx(center, center), 100);  // obstacle stays at max cost
  EXPECT_EQ(idx(0, 0), 0);              // 遠方は0コスト
  EXPECT_LT(idx(0, 0), idx(center, center));
  EXPECT_GT(idx(center, center - 1), idx(0, 0));  // 隣接セルは高コスト
  EXPECT_LT(idx(center, center - 1), 100);
}
