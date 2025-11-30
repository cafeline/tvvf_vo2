#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include "tvvf_vo_c/ros/tvvf_vo_node.hpp"

namespace tvvf_vo_c
{
class ObstaclesUnifiedTest : public ::testing::Test
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
}  // namespace tvvf_vo_c

using tvvf_vo_c::ObstaclesUnifiedTest;
using tvvf_vo_c::TVVFVONode;

TEST_F(ObstaclesUnifiedTest, CombinesMapAndMask)
{
  auto node = std::make_shared<TVVFVONode>();
  node->set_parameter(rclcpp::Parameter("obstacle_mask_topic", "/obstacle_mask"));
  node->set_parameter(rclcpp::Parameter("occupancy_clear_radius", 0.5));

  nav_msgs::msg::OccupancyGrid map;
  map.header.frame_id = "map";
  map.info.resolution = 1.0;
  map.info.width = 3;
  map.info.height = 3;
  map.info.origin.position.x = 0.0;
  map.info.origin.position.y = 0.0;
  map.info.origin.orientation.w = 1.0;
  map.data.assign(9, -1);

  nav_msgs::msg::OccupancyGrid mask = map;
  mask.data = {
    0, 0, 0,
    0, 100, 0,
    0, 0, 0
  };

  node->debug_set_map(map);
  node->debug_set_obstacle_mask(mask);
  node->debug_set_robot_position(tvvf_vo_c::Position(1.5, 1.5));

  auto fused = node->debug_build_combined_map();
  ASSERT_TRUE(fused.has_value());
  auto idx = [](uint32_t x, uint32_t y) {return static_cast<size_t>(y) * 3 + x;};

  EXPECT_EQ(fused->data[idx(1, 1)], 0);    // clear radius prevents mask overwrite at robot pos
  EXPECT_EQ(fused->data[idx(0, 0)], 0);    // free cell propagated into unknown map
}
