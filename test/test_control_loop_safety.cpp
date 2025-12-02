#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <thread>

#include "tvvf_vo_c/ros/tvvf_vo_node.hpp"

using tvvf_vo_c::TVVFVONode;

class ControlLoopSafetyTest : public ::testing::Test
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

  void SetUp() override
  {
    received_ = false;
    executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    listener_node_ = std::make_shared<rclcpp::Node>("cmd_listener");
    sub_ = listener_node_->create_subscription<geometry_msgs::msg::Twist>(
        "cmd_vel_raw", 10,
        [this](const geometry_msgs::msg::Twist::SharedPtr msg)
        {
          std::lock_guard<std::mutex> lock(mutex_);
          last_cmd_ = *msg;
          received_ = true;
          cv_.notify_all();
        });
    executor_->add_node(listener_node_);
  }

  void TearDown() override
  {
    executor_->cancel();
    executor_->remove_node(listener_node_);
  }

  bool wait_for_cmd(std::chrono::milliseconds timeout = std::chrono::milliseconds(200))
  {
    std::unique_lock<std::mutex> lock(mutex_);
    return cv_.wait_for(lock, timeout, [this] { return received_; });
  }

  geometry_msgs::msg::Twist last_cmd_{};
  bool received_{false};
  std::mutex mutex_;
  std::condition_variable cv_;
  rclcpp::executors::SingleThreadedExecutor::SharedPtr executor_;
  rclcpp::Node::SharedPtr listener_node_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_;
};

TEST_F(ControlLoopSafetyTest, PublishesStopWhenTfUnavailable)
{
  auto node = std::make_shared<TVVFVONode>();
  executor_->add_node(node);

  node->control_loop();
  executor_->spin_some();

  ASSERT_TRUE(wait_for_cmd());
  EXPECT_DOUBLE_EQ(last_cmd_.linear.x, 0.0);
  EXPECT_DOUBLE_EQ(last_cmd_.linear.y, 0.0);
  EXPECT_DOUBLE_EQ(last_cmd_.angular.z, 0.0);
}

TEST_F(ControlLoopSafetyTest, PublishesStopWhenGoalMissingWithValidTf)
{
  using namespace std::chrono_literals;

  // ブロードキャスト用の静的TFを用意（map -> base_footprint）
  auto tf_node = std::make_shared<rclcpp::Node>("tf_broadcaster_node");
  auto broadcaster = std::make_shared<tf2_ros::StaticTransformBroadcaster>(tf_node);
  geometry_msgs::msg::TransformStamped tf;
  tf.header.frame_id = "map";
  tf.child_frame_id = "base_footprint";
  tf.transform.translation.x = 0.0;
  tf.transform.translation.y = 0.0;
  tf.transform.translation.z = 0.0;
  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, 0.0);
  tf.transform.rotation.x = q.x();
  tf.transform.rotation.y = q.y();
  tf.transform.rotation.z = q.z();
  tf.transform.rotation.w = q.w();
  broadcaster->sendTransform(tf);

  auto node = std::make_shared<TVVFVONode>();

  executor_->add_node(tf_node);
  executor_->add_node(node);
  executor_->spin_some();
  std::this_thread::sleep_for(50ms);
  executor_->spin_some();

  node->control_loop();
  executor_->spin_some();

  ASSERT_TRUE(wait_for_cmd());
  EXPECT_DOUBLE_EQ(last_cmd_.linear.x, 0.0);
  EXPECT_DOUBLE_EQ(last_cmd_.linear.y, 0.0);
  EXPECT_DOUBLE_EQ(last_cmd_.angular.z, 0.0);
}
