#include "tvvf_vo_c/ros/tvvf_vo_node.hpp"
#include <algorithm>

namespace tvvf_vo_c
{

  void TVVFVONode::clicked_point_callback(const geometry_msgs::msg::PointStamped::SharedPtr msg)
  {
    Position goal_position(msg->point.x, msg->point.y);
    goal_ = Goal(goal_position, cached_params_.goal_tolerance);

    RCLCPP_INFO(this->get_logger(), "Goal set from clicked_point: (%.2f, %.2f)",
                goal_position.x, goal_position.y);
  }

  void TVVFVONode::goal_pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    Position goal_position(msg->pose.position.x, msg->pose.position.y);
    goal_ = Goal(goal_position, cached_params_.goal_tolerance);

    RCLCPP_INFO(this->get_logger(), "Goal set from goal_pose: (%.2f, %.2f)",
                goal_position.x, goal_position.y);

    if (!use_rviz_ || !goal_marker_pub_) {
      return;
    }

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
  }

  void TVVFVONode::obstacle_mask_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
  {
    obstacle_mask_ = *msg;
  }

  void TVVFVONode::refresh_map_obstacle_cache(const Position& robot_pos)
  {
    (void)robot_pos;
  }

} // namespace tvvf_vo_c
