#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>

#include "tvvf_vo_c/ros/tvvf_vo_node.hpp"

namespace
{
class SpeedFloorTest : public ::testing::Test
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

TEST_F(SpeedFloorTest, AppliesFloorWhenCellSpeedIsZeroOrNegative)
{
  rclcpp::NodeOptions opts;
  opts.append_parameter_override("max_linear_velocity", 1.0);
  opts.append_parameter_override("turning_linear_speed_mps", 0.2);
  auto node = std::make_shared<tvvf_vo_c::TVVFVONode>(opts);

  EXPECT_NEAR(node->debug_compute_speed_limit(0.0), 0.2, 1e-6);
  EXPECT_NEAR(node->debug_compute_speed_limit(-1.0), 0.2, 1e-6);
}

TEST_F(SpeedFloorTest, ClampsToFloorWhenBelowScale)
{
  rclcpp::NodeOptions opts;
  opts.append_parameter_override("max_linear_velocity", 1.0);
  opts.append_parameter_override("turning_linear_speed_mps", 0.2);
  auto node = std::make_shared<tvvf_vo_c::TVVFVONode>(opts);

  EXPECT_NEAR(node->debug_compute_speed_limit(0.05), 0.2, 1e-6);
}

TEST_F(SpeedFloorTest, PassesThroughWhenWithinBounds)
{
  rclcpp::NodeOptions opts;
  opts.append_parameter_override("max_linear_velocity", 1.0);
  opts.append_parameter_override("turning_linear_speed_mps", 0.2);
  auto node = std::make_shared<tvvf_vo_c::TVVFVONode>(opts);

  EXPECT_NEAR(node->debug_compute_speed_limit(0.6), 0.6, 1e-6);
}

TEST_F(SpeedFloorTest, CapsAtMaxWhenAboveMax)
{
  rclcpp::NodeOptions opts;
  opts.append_parameter_override("max_linear_velocity", 1.0);
  opts.append_parameter_override("turning_linear_speed_mps", 0.2);
  auto node = std::make_shared<tvvf_vo_c::TVVFVONode>(opts);

  EXPECT_NEAR(node->debug_compute_speed_limit(2.0), 1.0, 1e-6);
}
