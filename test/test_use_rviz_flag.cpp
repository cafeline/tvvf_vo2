#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include "tvvf_vo_c/ros/tvvf_vo_node.hpp"
#include "tvvf_vo_c/core/field_types.hpp"

#include <atomic>
#include <thread>

namespace
{
class UseRvizFlagTest : public ::testing::Test
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

  static tvvf_vo_c::VectorField make_single_cell_field()
  {
    tvvf_vo_c::VectorField field;
    field.resize(1, 1);
    field.resolution = 1.0;
    field.set_origin(tvvf_vo_c::Position(0.0, 0.0));
    field.vectors[0][0] = {1.0, 0.0};
    return field;
  }
};
}  // namespace

TEST_F(UseRvizFlagTest, DoesNotPublishWhenUseRvizIsFalse)
{
  rclcpp::NodeOptions opts;
  opts.append_parameter_override("use_rviz", false);
  auto node = std::make_shared<tvvf_vo_c::TVVFVONode>(opts);
  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(node);

  std::atomic<int> received{0};
  auto sub = node->create_subscription<geometry_msgs::msg::PoseArray>(
    "tvvf_vo_vector_pose_array",
    rclcpp::QoS(10).best_effort(),
    [&](const geometry_msgs::msg::PoseArray &) { received.fetch_add(1); });
  (void)sub;

  exec.spin_some();
  std::this_thread::sleep_for(std::chrono::milliseconds(50));

  auto field = make_single_cell_field();
  node->publish_combined_field_visualization(field);
  exec.spin_some();
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  exec.spin_some();

  EXPECT_EQ(received.load(), 0);
}

TEST_F(UseRvizFlagTest, DoesNotPublishGoalMarkerWhenUseRvizIsFalse)
{
  rclcpp::NodeOptions opts;
  opts.append_parameter_override("use_rviz", false);
  auto node = std::make_shared<tvvf_vo_c::TVVFVONode>(opts);
  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(node);

  std::atomic<int> received{0};
  auto sub = node->create_subscription<visualization_msgs::msg::Marker>(
    "goal_marker",
    rclcpp::QoS(10),
    [&](const visualization_msgs::msg::Marker &) { received.fetch_add(1); });
  (void)sub;

  exec.spin_some();
  std::this_thread::sleep_for(std::chrono::milliseconds(50));

  auto goal_msg = std::make_shared<geometry_msgs::msg::PoseStamped>();
  goal_msg->pose.position.x = 1.0;
  goal_msg->pose.position.y = 2.0;
  goal_msg->pose.orientation.w = 1.0;
  node->goal_pose_callback(goal_msg);

  exec.spin_some();
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  exec.spin_some();

  EXPECT_EQ(received.load(), 0);
}

TEST_F(UseRvizFlagTest, PublishesGoalMarkerWhenUseRvizIsTrue)
{
  rclcpp::NodeOptions opts;
  opts.append_parameter_override("use_rviz", true);
  auto node = std::make_shared<tvvf_vo_c::TVVFVONode>(opts);
  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(node);

  std::atomic<int> received{0};
  auto sub = node->create_subscription<visualization_msgs::msg::Marker>(
    "goal_marker",
    rclcpp::QoS(10),
    [&](const visualization_msgs::msg::Marker &) { received.fetch_add(1); });
  (void)sub;

  exec.spin_some();
  std::this_thread::sleep_for(std::chrono::milliseconds(50));

  auto goal_msg = std::make_shared<geometry_msgs::msg::PoseStamped>();
  goal_msg->pose.position.x = 1.0;
  goal_msg->pose.position.y = 2.0;
  goal_msg->pose.orientation.w = 1.0;
  node->goal_pose_callback(goal_msg);

  exec.spin_some();
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  exec.spin_some();

  EXPECT_GE(received.load(), 1);
}

TEST_F(UseRvizFlagTest, DoesNotPublishCommandMarkerWhenUseRvizIsFalse)
{
  rclcpp::NodeOptions opts;
  opts.append_parameter_override("use_rviz", false);
  auto node = std::make_shared<tvvf_vo_c::TVVFVONode>(opts);
  node->debug_set_robot_position(tvvf_vo_c::Position(0.0, 0.0));
  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(node);

  std::atomic<int> received{0};
  auto sub = node->create_subscription<visualization_msgs::msg::Marker>(
    "cmd_vel_marker",
    rclcpp::QoS(10),
    [&](const visualization_msgs::msg::Marker &) { received.fetch_add(1); });
  (void)sub;

  exec.spin_some();
  std::this_thread::sleep_for(std::chrono::milliseconds(50));

  tvvf_vo_c::ControlOutput output(
    tvvf_vo_c::Velocity(0.1, 0.0),
    0.0,
    0.05,
    0.0);
  node->publish_control_command(output);

  exec.spin_some();
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  exec.spin_some();

  EXPECT_EQ(received.load(), 0);
}

TEST_F(UseRvizFlagTest, PublishesCommandMarkerWhenUseRvizIsTrue)
{
  rclcpp::NodeOptions opts;
  opts.append_parameter_override("use_rviz", true);
  auto node = std::make_shared<tvvf_vo_c::TVVFVONode>(opts);
  node->debug_set_robot_position(tvvf_vo_c::Position(0.0, 0.0));
  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(node);

  std::atomic<int> received{0};
  auto sub = node->create_subscription<visualization_msgs::msg::Marker>(
    "cmd_vel_marker",
    rclcpp::QoS(10),
    [&](const visualization_msgs::msg::Marker &) { received.fetch_add(1); });
  (void)sub;

  exec.spin_some();
  std::this_thread::sleep_for(std::chrono::milliseconds(50));

  tvvf_vo_c::ControlOutput output(
    tvvf_vo_c::Velocity(0.1, 0.0),
    0.0,
    0.05,
    0.0);
  node->publish_control_command(output);

  exec.spin_some();
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  exec.spin_some();

  EXPECT_GE(received.load(), 1);
}

TEST_F(UseRvizFlagTest, PublishesWhenUseRvizIsTrue)
{
  rclcpp::NodeOptions opts;
  opts.append_parameter_override("use_rviz", true);
  auto node = std::make_shared<tvvf_vo_c::TVVFVONode>(opts);
  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(node);

  std::atomic<int> received{0};
  auto sub = node->create_subscription<geometry_msgs::msg::PoseArray>(
    "tvvf_vo_vector_pose_array",
    rclcpp::QoS(10).best_effort(),
    [&](const geometry_msgs::msg::PoseArray &) { received.fetch_add(1); });
  (void)sub;

  exec.spin_some();
  std::this_thread::sleep_for(std::chrono::milliseconds(50));

  auto field = make_single_cell_field();
  node->publish_combined_field_visualization(field);
  exec.spin_some();
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  exec.spin_some();

  EXPECT_GE(received.load(), 1);
}
