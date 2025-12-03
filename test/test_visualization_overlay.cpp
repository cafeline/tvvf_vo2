#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>

#include "tvvf_vo_c/ros/tvvf_vo_node.hpp"
#include "tvvf_vo_c/core/cost_map_builder.hpp"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

using namespace tvvf_vo_c;

class VisualizationOverlayTest : public ::testing::Test
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

TEST_F(VisualizationOverlayTest, CostmapGridUsesOverlayResolutionAndSpeed)
{
  auto node = std::make_shared<TVVFVONode>();

  CostMapResult base;
  base.width = 2;
  base.height = 2;
  base.resolution = 0.2;
  base.origin = Position(0.0, 0.0);
  base.speed_layer.assign(base.width * base.height, 1.0);
  base.clearance_layer.assign(base.width * base.height, 5.0);

  CostMapLayer overlay;
  overlay.width = 2;
  overlay.height = 2;
  overlay.resolution = 0.1;
  overlay.origin = Position(0.0, 0.0);
  overlay.speed_layer.assign(overlay.width * overlay.height, 0.0);  // block area
  overlay.clearance_layer.assign(overlay.width * overlay.height, 0.0);
  base.overlays.push_back(overlay);

  VectorField dummy_field;
  dummy_field.width = static_cast<int>(base.width);
  dummy_field.height = static_cast<int>(base.height);
  dummy_field.resolution = base.resolution;
  dummy_field.origin = Position(0.0, 0.0);

  auto grid = node->debug_build_costmap_grid(base, dummy_field, "map");
  ASSERT_NEAR(grid.info.resolution, 0.1, 1e-6);
  ASSERT_EQ(grid.info.width, 4u);
  ASSERT_EQ(grid.info.height, 4u);

  // overlay covers origin area, should be max cost
  EXPECT_EQ(grid.data[0], 100);
  // far corner should remain free (0 cost)
  const size_t far_idx = static_cast<size_t>(grid.info.height - 1) * grid.info.width +
                         static_cast<size_t>(grid.info.width - 1);
  EXPECT_EQ(grid.data[far_idx], 0);
}

TEST_F(VisualizationOverlayTest, VectorPoseArraySamplesOverlayResolution)
{
  auto node = std::make_shared<TVVFVONode>();

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

  auto pose_array = node->debug_build_vector_pose_array(base);
  ASSERT_EQ(pose_array.poses.size(), 7u);  // base 3 + overlay 4

  // Overlay area (near origin) should have yaw ~ +pi/2
  bool found_overlay_dir = false;
  for (const auto& pose : pose_array.poses) {
    if (pose.position.x < 0.15 && pose.position.y < 0.15) {
      tf2::Quaternion q;
      tf2::fromMsg(pose.orientation, q);
      double roll = 0.0, pitch = 0.0, yaw = 0.0;
      tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
      EXPECT_NEAR(yaw, M_PI_2, 1e-3);
      found_overlay_dir = true;
      break;
    }
  }
  EXPECT_TRUE(found_overlay_dir);
}

TEST_F(VisualizationOverlayTest, VectorPoseArrayUsesLocalResolutionWhenNoOverlay)
{
  rclcpp::NodeOptions opts;
  opts.append_parameter_override("local_costmap_resolution", 0.1);
  auto node = std::make_shared<TVVFVONode>(opts);

  VectorField base;
  base.resize(2, 2);
  base.resolution = 0.2;
  base.origin = Position(0.0, 0.0);
  for (int y = 0; y < base.height; ++y) {
    for (int x = 0; x < base.width; ++x) {
      base.vectors[static_cast<size_t>(y)][static_cast<size_t>(x)] = {1.0, 0.0};
    }
  }

  auto pose_array = node->debug_build_vector_pose_array(base);
  // width=2,height=2,res=0.2 => span 0.4m, with local_res 0.1 => 4x4 samples =16
  EXPECT_EQ(pose_array.poses.size(), 16u);
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
