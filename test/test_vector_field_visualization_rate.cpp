#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_array.hpp>

#include "tvvf_vo_c/core/field_types.hpp"
#include "tvvf_vo_c/ros/tvvf_vo_node.hpp"

#include <atomic>
#include <chrono>
#include <thread>

namespace
{
class VectorFieldVisualizationRateTest : public ::testing::Test
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

TEST_F(VectorFieldVisualizationRateTest, PublishesAtMostTwicePerSecond)
{
  auto node = std::make_shared<tvvf_vo_c::TVVFVONode>();
  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(node);

  std::atomic<int> received{0};
  auto sub = node->create_subscription<geometry_msgs::msg::PoseArray>(
    "tvvf_vo_vector_pose_array",
    rclcpp::QoS(10).best_effort(),
    [&](const geometry_msgs::msg::PoseArray &) { received.fetch_add(1); });

  // Allow discovery to settle
  exec.spin_some();
  std::this_thread::sleep_for(std::chrono::milliseconds(50));

  // ベクトル場を1セルだけ用意
  tvvf_vo_c::VectorField field;
  field.resize(1, 1);
  field.resolution = 1.0;
  field.set_origin(tvvf_vo_c::Position(0.0, 0.0));
  field.vectors[0][0] = {1.0, 0.0};

  // 1回目のpublish
  node->publish_combined_field_visualization(field);
  exec.spin_some();
  EXPECT_EQ(received.load(), 1);

  // 0.5秒未満ではpublishされない
  node->publish_combined_field_visualization(field);
  exec.spin_some();
  EXPECT_EQ(received.load(), 1);

  // 0.5秒以上経過するとpublishされる
  std::this_thread::sleep_for(std::chrono::milliseconds(600));
  node->publish_combined_field_visualization(field);
  exec.spin_some();
  EXPECT_EQ(received.load(), 2);
}
