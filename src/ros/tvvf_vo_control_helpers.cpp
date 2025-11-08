#include "tvvf_vo_c/ros/tvvf_vo_node.hpp"

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

    // 基本のベクトル場から速度を取得
    auto velocity_vector = global_field_generator_->getVelocityAt(
        robot_state_->position, dynamic_obstacles_);

    // 静的障害物からの斥力を追加
    apply_repulsive_force(velocity_vector);

    // 速度をスケーリング
    scale_velocity_vector(velocity_vector);

    // 制御出力を作成
    return ControlOutput(
        Velocity(velocity_vector[0], velocity_vector[1]),
        0.01,  // computation_time
        0.01,  // dt
        1.0    // confidence
    );
  }

  void TVVFVONode::apply_repulsive_force(std::array<double, 2>& velocity_vector)
  {
    if (static_obstacle_cache_ready_ && repulsive_force_calculator_) {
      const auto repulsive_force = repulsive_force_calculator_->calculateTotalForceFromPositions(
          robot_state_->position, static_obstacle_positions_cache_);
      velocity_vector[0] += repulsive_force.x;
      velocity_vector[1] += repulsive_force.y;
    }
  }

  void TVVFVONode::scale_velocity_vector(std::array<double, 2>& velocity_vector)
  {
    const double max_velocity = this->get_parameter("max_linear_velocity").as_double();
    velocity_vector[0] *= max_velocity;
    velocity_vector[1] *= max_velocity;
  }

  void TVVFVONode::update_visualization()
  {
    if (this->count_subscribers("tvvf_vo_vector_field") == 0) {
      return;
    }

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
