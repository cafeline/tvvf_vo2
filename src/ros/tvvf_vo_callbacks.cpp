#include "tvvf_vo_c/ros/tvvf_vo_node.hpp"
#include <algorithm>

namespace tvvf_vo_c
{

  void TVVFVONode::clicked_point_callback(const geometry_msgs::msg::PointStamped::SharedPtr msg)
  {
    Position goal_position(msg->point.x, msg->point.y);
    goal_ = Goal(goal_position, this->get_parameter("goal_tolerance").as_double());

    // GlobalFieldGeneratorで静的場を再計算
    if (current_map_.has_value() && global_field_generator_) {
      global_field_generator_->precomputeStaticField(current_map_.value(), goal_position);
    } else {
      if (!current_map_.has_value()) {
        RCLCPP_WARN(this->get_logger(), "No map available for static field computation");
      }
      if (!global_field_generator_) {
        RCLCPP_ERROR(this->get_logger(), "GlobalFieldGenerator not initialized");
      }
    }

    RCLCPP_INFO(this->get_logger(), "Goal set from clicked_point: (%.2f, %.2f)",
                goal_position.x, goal_position.y);
  }

  void TVVFVONode::goal_pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    Position goal_position(msg->pose.position.x, msg->pose.position.y);
    goal_ = Goal(goal_position, this->get_parameter("goal_tolerance").as_double());

    // GlobalFieldGeneratorで静的場を再計算
    if (current_map_.has_value() && global_field_generator_) {
      global_field_generator_->precomputeStaticField(current_map_.value(), goal_position);
    } else {
      if (!current_map_.has_value()) {
        RCLCPP_WARN(this->get_logger(), "No map available for static field computation");
      }
      if (!global_field_generator_) {
        RCLCPP_ERROR(this->get_logger(), "GlobalFieldGenerator not initialized");
      }
    }

    RCLCPP_INFO(this->get_logger(), "Goal set from goal_pose: (%.2f, %.2f)",
                goal_position.x, goal_position.y);

    // ゴールマーカーを可視化
    auto goal_marker = visualization_msgs::msg::Marker();
    goal_marker.header.frame_id = "map";
    goal_marker.header.stamp = this->now();
    goal_marker.ns = "goal";
    goal_marker.id = 0;
    goal_marker.type = visualization_msgs::msg::Marker::SPHERE;
    goal_marker.action = visualization_msgs::msg::Marker::ADD;
    goal_marker.pose.position.x = goal_position.x;
    goal_marker.pose.position.y = goal_position.y;
    goal_marker.pose.position.z = 0.3;
    goal_marker.pose.orientation.w = 1.0;
    goal_marker.scale.x = 0.5;
    goal_marker.scale.y = 0.5;
    goal_marker.scale.z = 0.5;
    goal_marker.color.r = 1.0;
    goal_marker.color.g = 0.0;
    goal_marker.color.b = 0.0;
    goal_marker.color.a = 0.8;
    goal_marker_pub_->publish(goal_marker);
  }

  void TVVFVONode::map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
  {
    
    // マップを保存
    current_map_ = *msg;
    
    // GlobalFieldGeneratorで静的場を再計算（ゴールが設定されている場合）
    if (goal_.has_value() && global_field_generator_) {
      global_field_generator_->precomputeStaticField(*msg, goal_->position);
    }
  }

  void TVVFVONode::obstacles_callback(const visualization_msgs::msg::MarkerArray::SharedPtr msg, bool is_dynamic)
  {
    if (!is_dynamic) {
      // 静的障害物はマップに含まれるため無視
      return;
    }
    
    dynamic_obstacles_.clear();

    for (const auto &marker : msg->markers)
    {
      if (marker.action == visualization_msgs::msg::Marker::ADD)
      {
        Position position(marker.pose.position.x, marker.pose.position.y);
        Velocity velocity(0.0, 0.0);
        double radius = std::max(marker.scale.x, marker.scale.y) / 2.0;
        // DynamicObstacle構造体は3つの引数を取る（IDは持たない）
        dynamic_obstacles_.emplace_back(position, velocity, radius);
      }
    }
  }

} // namespace tvvf_vo_c