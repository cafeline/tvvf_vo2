#include "tvvf_vo_c/ros/tvvf_vo_node.hpp"
#include <algorithm>

namespace tvvf_vo_c
{

  void TVVFVONode::clicked_point_callback(const geometry_msgs::msg::PointStamped::SharedPtr msg)
  {
    Position goal_position(msg->point.x, msg->point.y);
    goal_ = Goal(goal_position, this->get_parameter("goal_tolerance").as_double());
    map_obstacles_dirty_ = true;

    request_static_field_update();
    try_recompute_static_field();

    RCLCPP_INFO(this->get_logger(), "Goal set from clicked_point: (%.2f, %.2f)",
                goal_position.x, goal_position.y);
  }

  void TVVFVONode::goal_pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    Position goal_position(msg->pose.position.x, msg->pose.position.y);
    goal_ = Goal(goal_position, this->get_parameter("goal_tolerance").as_double());
    map_obstacles_dirty_ = true;

    request_static_field_update();
    try_recompute_static_field();

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
    map_obstacles_dirty_ = true;

    // GlobalFieldGeneratorで静的場を再計算（ゴールが設定されている場合）
    if (goal_.has_value() && global_field_generator_) {
      request_static_field_update();
      try_recompute_static_field();
    }
  }

  void TVVFVONode::static_obstacles_callback(const visualization_msgs::msg::MarkerArray::SharedPtr msg)
  {
    external_static_obstacles_ = *msg;
    update_combined_static_obstacles();
  }

  void TVVFVONode::update_static_obstacle_cache(const visualization_msgs::msg::MarkerArray& msg)
  {
    if (repulsive_force_calculator_ && !msg.markers.empty()) {
      static_obstacle_positions_cache_ = repulsive_force_calculator_->extractObstaclePositions(msg);
      static_obstacle_cache_ready_ = !static_obstacle_positions_cache_.empty();
    } else {
      static_obstacle_positions_cache_.clear();
      static_obstacle_cache_ready_ = false;
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

  void TVVFVONode::update_combined_static_obstacles()
  {
    visualization_msgs::msg::MarkerArray combined;

    if (map_static_obstacles_.has_value()) {
      combined.markers.insert(
          combined.markers.end(),
          map_static_obstacles_->markers.begin(),
          map_static_obstacles_->markers.end());
    }

    if (external_static_obstacles_.has_value()) {
      combined.markers.insert(
          combined.markers.end(),
          external_static_obstacles_->markers.begin(),
          external_static_obstacles_->markers.end());
    }

    if (combined.markers.empty()) {
      static_obstacles_.reset();
      static_obstacle_positions_cache_.clear();
      static_obstacle_cache_ready_ = false;
      return;
    }

    static_obstacles_ = combined;
    update_static_obstacle_cache(combined);
  }

  void TVVFVONode::refresh_map_obstacle_cache(const Position& robot_pos)
  {
    if (!enable_map_repulsion_ || !current_map_.has_value() || !goal_.has_value()) {
      return;
    }

    const std::string frame_id = current_map_->header.frame_id.empty()
        ? this->get_parameter("global_frame").as_string()
        : current_map_->header.frame_id;

    auto publish_delete_all = [&](const std::string& frame) {
      visualization_msgs::msg::MarkerArray delete_array;
      visualization_msgs::msg::Marker marker;
      marker.header.frame_id = frame;
      marker.header.stamp = this->now();
      marker.action = visualization_msgs::msg::Marker::DELETEALL;
      marker.id = 0;
      delete_array.markers.push_back(marker);
      publish_map_obstacle_markers(delete_array);
    };

    if (!map_obstacles_dirty_) {
      if (last_map_repulsion_center_.has_value()) {
        const double dx = robot_pos.x - last_map_repulsion_center_->x;
        const double dy = robot_pos.y - last_map_repulsion_center_->y;
        if ((dx * dx + dy * dy) < map_repulsion_recompute_distance_sq_) {
          return;
        }
      } else {
        map_obstacles_dirty_ = true;
      }
    }

    auto region = build_planning_region();
    if (!region.has_value()) {
      map_static_obstacles_.reset();
      publish_delete_all(frame_id);
      update_combined_static_obstacles();
      return;
    }

    const auto obstacle_positions = extract_map_obstacle_positions(
        current_map_.value(), region.value(), robot_pos, map_repulsion_settings_);

    if (obstacle_positions.empty()) {
      map_static_obstacles_.reset();
      publish_delete_all(frame_id);
      last_map_repulsion_center_ = robot_pos;
      map_obstacles_dirty_ = false;
      update_combined_static_obstacles();
      return;
    }

    const double scale = std::max(
        map_repulsion_settings_.sampling_step,
        static_cast<double>(current_map_->info.resolution));

    map_static_obstacles_ = create_map_obstacle_markers(
        obstacle_positions, scale, frame_id, this->now());
    publish_map_obstacle_markers(map_static_obstacles_.value());

    last_map_repulsion_center_ = robot_pos;
    map_obstacles_dirty_ = false;
    update_combined_static_obstacles();
  }

  void TVVFVONode::publish_map_obstacle_markers(const visualization_msgs::msg::MarkerArray& markers)
  {
    if (!publish_map_obstacle_markers_ || !map_obstacles_marker_pub_) {
      return;
    }

    if (markers.markers.empty()) {
      return;
    }

    map_obstacles_marker_pub_->publish(markers);
  }

} // namespace tvvf_vo_c
