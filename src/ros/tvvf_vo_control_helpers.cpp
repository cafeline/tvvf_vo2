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
    field_update_pending_ = false;
    publish_empty_visualization();
    RCLCPP_INFO(this->get_logger(), "Goal reached");
  }

  ControlOutput TVVFVONode::compute_control_output()
  {
    // グローバルフィールドが準備できていない場合は停止
    if (!global_field_generator_ || !global_field_generator_->isStaticFieldReady()) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                          "Global field not ready - stopping");
      return ControlOutput(Velocity(0.0, 0.0), 0.01, 0.01, 0.0);
    }

    auto base_field_vector = global_field_generator_->getVelocityAt(
        robot_state_->position, dynamic_obstacles_);
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
    opt_state.dynamic_obstacles = dynamic_obstacles_;
    opt_state.static_obstacles.clear();  // 静的障害物は repulsive force に統合済み
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
    const auto now = this->get_clock()->now();
    if (!is_vector_field_publish_due(now)) {
      return;
    }

    const VectorField global_field = global_field_generator_->generateField(dynamic_obstacles_);
    if (global_field.width > 0 && global_field.height > 0) {
      publish_combined_field_visualization(global_field);
      last_vector_field_publish_time_ = now;
    }
  }

  bool TVVFVONode::is_vector_field_publish_due(const rclcpp::Time& now) const
  {
    if (vector_field_publish_interval_ <= 0.0) {
      return true;
    }

    if (last_vector_field_publish_time_.nanoseconds() == 0) {
      return true;
    }

    const double elapsed = (now - last_vector_field_publish_time_).seconds();
    return elapsed >= vector_field_publish_interval_;
  }

} // namespace tvvf_vo_c
