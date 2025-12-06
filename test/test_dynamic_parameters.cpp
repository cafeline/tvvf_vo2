#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>

#include "tvvf_vo_c/ros/tvvf_vo_node.hpp"

namespace
{
class DynamicParametersTest : public ::testing::Test
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
}  // namespace

TEST_F(DynamicParametersTest, UpdatesSpeedRelatedParametersAtRuntime)
{
  rclcpp::NodeOptions opts;
  opts.append_parameter_override("max_linear_velocity", 1.0);
  opts.append_parameter_override("turning_linear_speed_mps", 0.1);
  opts.append_parameter_override("costmap_min_speed", 0.05);
  opts.append_parameter_override("costmap_max_speed", 0.5);
  opts.append_parameter_override("costmap_escape_speed_min", 0.05);

  auto node = std::make_shared<tvvf_vo_c::TVVFVONode>(opts);

  auto results = node->set_parameters({
      rclcpp::Parameter("max_linear_velocity", 0.4),
      rclcpp::Parameter("costmap_min_speed", 0.2),
      rclcpp::Parameter("costmap_max_speed", 0.6),
      rclcpp::Parameter("costmap_escape_speed_min", 0.25),
      rclcpp::Parameter("turning_linear_speed_mps", 0.15),
  });

  for (const auto & res : results) {
    EXPECT_TRUE(res.successful);
  }

  const auto settings = node->debug_get_costmap_settings();
  EXPECT_NEAR(settings.min_speed, 0.2, 1e-6);
  EXPECT_NEAR(settings.max_speed, 0.6, 1e-6);
  EXPECT_NEAR(settings.escape_speed_min, 0.25, 1e-6);

  EXPECT_NEAR(node->debug_get_config().max_linear_velocity, 0.4, 1e-6);
  EXPECT_NEAR(node->debug_get_turning_linear_speed_mps(), 0.15, 1e-6);
  EXPECT_NEAR(node->debug_compute_speed_limit(0.0), 0.15, 1e-6);
  EXPECT_NEAR(node->debug_compute_speed_limit(1.0), 0.4, 1e-6);
}

TEST_F(DynamicParametersTest, ClampsEscapeSpeedIntoRange)
{
  rclcpp::NodeOptions opts;
  opts.append_parameter_override("costmap_min_speed", 0.1);
  opts.append_parameter_override("costmap_max_speed", 0.5);
  opts.append_parameter_override("costmap_escape_speed_min", 0.2);

  auto node = std::make_shared<tvvf_vo_c::TVVFVONode>(opts);

  auto results = node->set_parameters({
      rclcpp::Parameter("costmap_min_speed", 0.3),
      rclcpp::Parameter("costmap_max_speed", 0.2),   // smaller than min -> clamp up
      rclcpp::Parameter("costmap_escape_speed_min", 1.0),  // above max -> clamp down
  });
  for (const auto & res : results) {
    EXPECT_TRUE(res.successful);
  }

  const auto settings = node->debug_get_costmap_settings();
  EXPECT_NEAR(settings.min_speed, 0.3, 1e-6);
  EXPECT_NEAR(settings.max_speed, 0.3, 1e-6);            // clamped to min
  EXPECT_NEAR(settings.escape_speed_min, 0.3, 1e-6);     // clamped to [min,max]
}
