#include "tvvf_vo_c/ros/tvvf_vo_node.hpp"
#include <cmath>

namespace tvvf_vo_c
{
  bool TVVFVONode::update_robot_state()
  {
    robot_state_ = get_robot_pose_from_tf();
    if (!robot_state_.has_value()) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                          "Cannot get robot pose from TF");
      return false;
    }
    return true;
  }

  bool TVVFVONode::has_valid_goal() const
  {
    return goal_.has_value();
  }

  bool TVVFVONode::is_goal_reached() const
  {
    if (!robot_state_.has_value() || !goal_.has_value()) {
      return false;
    }
    const double distance_to_goal = robot_state_->position.distance_to(goal_->position);
    return distance_to_goal < goal_->tolerance;
  }

  void TVVFVONode::handle_goal_reached()
  {
    publish_stop_command();
    goal_.reset();
    publish_empty_visualization();
    RCLCPP_INFO(this->get_logger(), "Goal reached");
  }

  ControlOutput TVVFVONode::compute_control_output()
  {
    if (!global_field_generator_ || !current_map_.has_value() || !goal_.has_value()) {
      latest_field_.reset();
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                          "Map/goal not ready - stopping");
      return ControlOutput(Velocity(0.0, 0.0), 0.01, 0.01, 0.0);
    }

    auto map_to_use = build_combined_map();
    if (!map_to_use.has_value()) {
      latest_field_.reset();
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                          "Combined map unavailable - stopping");
      return ControlOutput(Velocity(0.0, 0.0), 0.01, 0.01, 0.0);
    }

    auto region = build_planning_region();
    latest_field_ = global_field_generator_->computeFieldOnTheFly(
        map_to_use.value(), goal_->position, region);

    if (!latest_field_.has_value() || latest_field_->width == 0 || latest_field_->height == 0) {
      latest_field_.reset();
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                          "Vector field generation failed - stopping");
      return ControlOutput(Velocity(0.0, 0.0), 0.01, 0.01, 0.0);
    }

    auto base_field_vector = latest_field_->getVector(robot_state_->position);
    auto field_vector = compute_navigation_vector(base_field_vector, robot_state_->position);

    const double vector_norm = std::hypot(field_vector[0], field_vector[1]);
    Velocity desired_velocity(0.0, 0.0);
    if (vector_norm > 1e-6) {
      desired_velocity.vx = field_vector[0] * config_.max_linear_velocity;
      desired_velocity.vy = field_vector[1] * config_.max_linear_velocity;
    }

    OptimizationState opt_state;
    opt_state.desired_velocity = desired_velocity;
    opt_state.previous_velocity = previous_velocity_command_.value_or(Velocity(0.0, 0.0));
    opt_state.robot_position = robot_state_->position;
    opt_state.max_speed = config_.max_linear_velocity;
    opt_state.dt = 0.05;

    Velocity optimized_velocity = desired_velocity;
    if (velocity_optimizer_) {
      optimized_velocity = velocity_optimizer_->computeOptimalVelocity(opt_state);
    }
    previous_velocity_command_ = optimized_velocity;

    return ControlOutput(
        optimized_velocity,
        0.01,
        0.05,
        1.0);
  }

  void TVVFVONode::update_visualization()
  {
    if (!latest_field_.has_value()) {
      return;
    }

    if (global_field_generator_) {
      const auto & cost_map = global_field_generator_->getLastCostMap();
      if (cost_map.has_value()) {
        const std::string frame_id = current_map_.has_value()
            ? current_map_->header.frame_id
            : this->get_parameter("global_frame").as_string();
        publish_costmap_visualization(*cost_map, *latest_field_, frame_id);
      }
    }

    if (latest_field_->width > 0 && latest_field_->height > 0) {
      publish_combined_field_visualization(*latest_field_);
    }
  }

  std::optional<nav_msgs::msg::OccupancyGrid> TVVFVONode::build_combined_map() const
  {
    if (!current_map_.has_value()) {
      return std::nullopt;
    }

    nav_msgs::msg::OccupancyGrid merged = *current_map_;
    const double clear_radius = std::max(
      0.0, this->get_parameter("occupancy_clear_radius").as_double());
    const bool has_robot_state = robot_state_.has_value();
    const double clear_r2 = clear_radius * clear_radius;

    if (has_robot_state && clear_radius > 0.0 &&
      merged.info.width > 0 && merged.info.height > 0)
    {
      const double res = std::max(static_cast<double>(merged.info.resolution), 1e-6);
      const double origin_x = merged.info.origin.position.x;
      const double origin_y = merged.info.origin.position.y;
      for (uint32_t row = 0; row < merged.info.height; ++row) {
        for (uint32_t col = 0; col < merged.info.width; ++col) {
          const double wx = origin_x + (static_cast<double>(col) + 0.5) * res;
          const double wy = origin_y + (static_cast<double>(row) + 0.5) * res;
          const double dx = wx - robot_state_->position.x;
          const double dy = wy - robot_state_->position.y;
          const size_t idx = static_cast<size_t>(row) * merged.info.width + col;
          const int8_t val = merged.data[idx];
          if (val < 0) {
            if (dx * dx + dy * dy <= clear_r2) {
              merged.data[idx] = 0;
            }
            continue;
          }
          if (dx * dx + dy * dy <= clear_r2) {
            merged.data[idx] = 0;
          }
        }
      }
    }

    if (!obstacle_mask_.has_value()) {
      const double override_res = this->get_parameter("costmap_resolution").as_double();
      if (override_res > 1e-6 &&
        std::abs(override_res - merged.info.resolution) > 1e-6)
      {
        merged = resample_occupancy_grid(merged, override_res);
      }
      return merged;
    }

    const auto & mask = obstacle_mask_.value();
    if (mask.info.width == 0 || mask.info.height == 0) {
      return merged;
    }

    if (mask.header.frame_id != merged.header.frame_id) {
      RCLCPP_WARN(
        this->get_logger(),
        "Obstacle mask frame_id (%s) differs from map (%s). Ignoring mask.",
        mask.header.frame_id.c_str(), merged.header.frame_id.c_str());
      return merged;
    }

    const double map_res = std::max(static_cast<double>(merged.info.resolution), 1e-6);
    const double inv_map_res = 1.0 / map_res;
    const double mask_res = std::max(static_cast<double>(mask.info.resolution), 1e-6);
    const double map_origin_x = merged.info.origin.position.x;
    const double map_origin_y = merged.info.origin.position.y;
    const double mask_origin_x = mask.info.origin.position.x;
    const double mask_origin_y = mask.info.origin.position.y;

    for (uint32_t row = 0; row < mask.info.height; ++row) {
      for (uint32_t col = 0; col < mask.info.width; ++col) {
        const size_t idx_mask = static_cast<size_t>(row) * mask.info.width + col;
        const int8_t val = mask.data[idx_mask];
        if (val < 0) {
          continue;
        }
        const double wx = mask_origin_x + (static_cast<double>(col) + 0.5) * mask_res;
        const double wy = mask_origin_y + (static_cast<double>(row) + 0.5) * mask_res;
        if (has_robot_state && clear_radius > 0.0) {
          const double dx = wx - robot_state_->position.x;
          const double dy = wy - robot_state_->position.y;
          if (dx * dx + dy * dy <= clear_r2) {
            continue;  // 足元セルはマスク適用しない
          }
        }
        const int map_col = static_cast<int>(std::floor((wx - map_origin_x) * inv_map_res));
        const int map_row = static_cast<int>(std::floor((wy - map_origin_y) * inv_map_res));
        if (map_col < 0 || map_row < 0 ||
            map_col >= static_cast<int>(merged.info.width) ||
            map_row >= static_cast<int>(merged.info.height)) {
          continue;
        }
        const size_t idx_map =
          static_cast<size_t>(map_row) * merged.info.width + static_cast<size_t>(map_col);
        if (val >= 100) {
          merged.data[idx_map] = 100;
        } else if (val == 0 && merged.data[idx_map] < 0) {
          merged.data[idx_map] = 0;
        }
      }
    }

    const double override_res = this->get_parameter("costmap_resolution").as_double();
    if (override_res > 1e-6 &&
      std::abs(override_res - merged.info.resolution) > 1e-6)
    {
      merged = resample_occupancy_grid(merged, override_res);
    }
    return merged;
  }

} // namespace tvvf_vo_c
