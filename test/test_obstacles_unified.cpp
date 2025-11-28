#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "tvvf_vo_c/ros/tvvf_vo_node.hpp"

using namespace tvvf_vo_c;

namespace {
visualization_msgs::msg::Marker make_cube(double x, double y)
{
  visualization_msgs::msg::Marker m;
  m.action = visualization_msgs::msg::Marker::ADD;
  m.type = visualization_msgs::msg::Marker::CUBE;
  m.pose.position.x = x;
  m.pose.position.y = y;
  m.scale.x = 1.0;
  m.scale.y = 1.0;
  m.scale.z = 0.1;
  m.color.a = 1.0;
  return m;
}

visualization_msgs::msg::Marker make_line_list_square()
{
  visualization_msgs::msg::Marker m;
  m.action = visualization_msgs::msg::Marker::ADD;
  m.type = visualization_msgs::msg::Marker::LINE_LIST;
  m.scale.x = 0.05;
  m.color.a = 1.0;
  std::vector<std::pair<double, double>> corners = {
      {-0.5, -0.5}, {0.5, -0.5}, {0.5, 0.5}, {-0.5, 0.5}};
  for (size_t i = 0; i < corners.size(); ++i) {
    const auto &a = corners[i];
    const auto &b = corners[(i + 1) % corners.size()];
    geometry_msgs::msg::Point p1, p2;
    p1.x = a.first; p1.y = a.second;
    p2.x = b.first; p2.y = b.second;
    m.points.push_back(p1);
    m.points.push_back(p2);
  }
  return m;
}
}  // namespace

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

TEST_F(ObstaclesUnifiedTest, ProcessesMergedObstacles)
{
  auto node = std::make_shared<TVVFVONode>();

  visualization_msgs::msg::MarkerArray msg;
  msg.markers.push_back(make_cube(1.0, 0.0));
  msg.markers.push_back(make_line_list_square());

  node->debug_handle_obstacles(std::make_shared<visualization_msgs::msg::MarkerArray>(msg));

  ASSERT_EQ(node->debug_dynamic_obstacle_count(), 2u);
  EXPECT_TRUE(node->debug_static_obstacle_cache_ready());
  ASSERT_EQ(node->debug_static_hull_count(), 1u);
  const auto hull = node->debug_static_hull_at(0);
  EXPECT_NEAR(hull.centroid.x, 0.0, 1e-9);
  EXPECT_NEAR(hull.centroid.y, 0.0, 1e-9);
}
