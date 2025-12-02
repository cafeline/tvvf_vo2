#include "tvvf_vo_c/ros/tvvf_vo_node.hpp"
#include "tvvf_vo_c/utils/math_utils.hpp"
#include <cmath>
#include <algorithm>

namespace tvvf_vo_c
{

  void TVVFVONode::control_loop()
  {
    try
    {
      const bool robot_ok = update_robot_state();

      if (!robot_ok) {
        publish_stop_command();
        // ロボット姿勢が取れなくても可視化は進める
        update_visualization();
        return;
      }

      // ロボット状態の更新とチェック
      refresh_map_obstacle_cache(robot_state_->position);

      // ゴールのチェック
      if (!has_valid_goal()) {
        publish_stop_command();
        return;
      }

      // 目標到達チェック
      if (is_goal_reached()) {
        handle_goal_reached();
        return;
      }

      // 制御出力を計算
      const auto control_output = compute_control_output();
      
      // 可視化を更新
      update_visualization();
      publish_planned_path();

      // 制御コマンドを発行
      publish_control_command(control_output);
    }
    catch (const std::exception &e)
    {
      RCLCPP_ERROR(this->get_logger(), "Control loop error: %s", e.what());
      publish_stop_command();
    }
  }

  void TVVFVONode::publish_control_command(const ControlOutput &control_output)
  {
    // 差動駆動用：desired_vx, desired_vyを変換
    double desired_vx = control_output.velocity_command.vx;
    double desired_vy = control_output.velocity_command.vy;

    auto cmd_msg = geometry_msgs::msg::Twist();

    // 目標ベクトルがゼロなら即停止を出して回頭を防ぐ
    if (std::abs(desired_vx) < 1e-6 && std::abs(desired_vy) < 1e-6) {
      cmd_vel_pub_->publish(cmd_msg);
      publish_command_marker(cmd_msg);
      return;
    }

    // 差動駆動ロボットでは横移動不可能なので、変換関数を使用
    auto [linear_vel, angular_vel] = convert_to_differential_drive(
        desired_vx, desired_vy, robot_state_->orientation);

    cmd_msg.linear.x = linear_vel;
    cmd_msg.linear.y = 0.0;  // 差動駆動では常に0
    cmd_msg.angular.z = angular_vel;

    // 速度制限（convert_to_differential_drive内で既に考慮されているが、念のため）
    double max_linear_vel = this->get_parameter("max_linear_velocity").as_double();
    double max_angular_vel = this->get_parameter("max_angular_velocity").as_double();

    cmd_msg.linear.x = std::clamp(cmd_msg.linear.x, -max_linear_vel, max_linear_vel);
    cmd_msg.angular.z = std::clamp(cmd_msg.angular.z, -max_angular_vel, max_angular_vel);

    cmd_vel_pub_->publish(cmd_msg);
    publish_command_marker(cmd_msg);
  }

  std::pair<double, double> TVVFVONode::convert_to_differential_drive(
      double desired_vx, double desired_vy, double current_orientation)
  {
    // 目標速度ベクトルの大きさと方向
    double target_speed = std::sqrt(desired_vx * desired_vx + desired_vy * desired_vy);
    double target_angle = std::atan2(desired_vy, desired_vx);

    // 現在の姿勢との角度差
    double angle_diff = math_utils::normalize_angle(target_angle - current_orientation);

    const double orientation_tolerance = this->get_parameter("orientation_tolerance").as_double();
    const double turning_linear_scale = this->get_parameter("turning_linear_scale").as_double();
    const double turning_angular_gain = this->get_parameter("turning_angular_gain").as_double();
    const double tracking_angular_gain = this->get_parameter("tracking_angular_gain").as_double();

    // 角度差が大きい場合は回転を優先
    double linear_velocity, angular_velocity;
    if (std::abs(angle_diff) > orientation_tolerance)
    {
      // 回転優先モード
      linear_velocity = target_speed * std::cos(angle_diff) * turning_linear_scale;
      angular_velocity = turning_angular_gain * angle_diff;
    }
    else
    {
      // 前進優先モード
      linear_velocity = target_speed * std::cos(angle_diff);
      angular_velocity = tracking_angular_gain * angle_diff;
    }

    // 旋回時の後進を防ぐため、線速度は非負にクリップ
    if (linear_velocity < 0.0) {
      linear_velocity = 0.0;
    }

    return {linear_velocity, angular_velocity};
  }

  void TVVFVONode::publish_stop_command()
  {
    auto cmd_msg = geometry_msgs::msg::Twist();
    cmd_vel_pub_->publish(cmd_msg);
    previous_velocity_command_.reset();
  }

void TVVFVONode::publish_planned_path()
{
  if (!robot_state_.has_value() || !goal_.has_value() || !latest_field_.has_value()) {
    return;
  }

  nav_msgs::msg::Path path_msg;
  path_msg.header.frame_id = "map";
  path_msg.header.stamp = this->now();

  // 簡易パス：現在位置からゴールまでベクトル場に沿って統合
  Position current_pos = robot_state_->position;
  const int max_steps = 100;
  const double step_size = 0.1;

  for (int i = 0; i < max_steps; ++i) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = "map";
    pose.header.stamp = this->now();
    pose.pose.position.x = current_pos.x;
    pose.pose.position.y = current_pos.y;
    pose.pose.position.z = 0.0;
    pose.pose.orientation.w = 1.0;
    path_msg.poses.push_back(pose);

    // ゴール到達チェック
    double dist_to_goal = std::sqrt(
        std::pow(goal_->position.x - current_pos.x, 2) +
        std::pow(goal_->position.y - current_pos.y, 2));
    
    if (dist_to_goal < goal_->tolerance) {
      break;
    }

    auto field_vector = compute_navigation_vector(latest_field_->getVector(current_pos), current_pos);
    double norm = std::sqrt(field_vector[0] * field_vector[0] + field_vector[1] * field_vector[1]);

    if (norm > 0.001) {
      current_pos.x += (field_vector[0] / norm) * step_size;
      current_pos.y += (field_vector[1] / norm) * step_size;
    } else {
      break;
    }
  }

  planned_path_pub_->publish(path_msg);
}

  void TVVFVONode::publish_command_marker(const geometry_msgs::msg::Twist& cmd_msg)
  {
    if (!cmd_velocity_marker_pub_) {
      return;
    }

    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = base_frame_.empty()
        ? this->get_parameter("base_frame").as_string()
        : base_frame_;
    marker.header.stamp = this->now();
    marker.ns = "tvvf_cmd_velocity";
    marker.id = cmd_velocity_marker_seq_++;
    marker.type = visualization_msgs::msg::Marker::ARROW;
    marker.action = visualization_msgs::msg::Marker::ADD;

    geometry_msgs::msg::Point start, end;
    start.x = 0.0;
    start.y = 0.0;
    start.z = 0.1;
    end.x = std::clamp(cmd_msg.linear.x, -config_.max_linear_velocity, config_.max_linear_velocity);
    end.y = std::clamp(cmd_msg.linear.y, -config_.max_linear_velocity, config_.max_linear_velocity);
    end.z = 0.1;
    marker.points.push_back(start);
    marker.points.push_back(end);

    marker.scale.x = 0.03;
    marker.scale.y = 0.06;
    marker.scale.z = 0.08;

    const double magnitude = std::hypot(end.x, end.y);
    const double normalized = std::min(1.0, magnitude / std::max(config_.max_linear_velocity, 1e-3));
    if (end.x >= 0.0) {
      marker.color.r = 0.1;
      marker.color.g = 0.8 * normalized + 0.2;
      marker.color.b = 0.2;
    } else {
      marker.color.r = 0.8 * normalized + 0.2;
      marker.color.g = 0.2;
      marker.color.b = 0.2;
    }
    marker.color.a = 0.9;
    marker.lifetime = rclcpp::Duration::from_seconds(0.2);

    cmd_velocity_marker_pub_->publish(marker);
  }

} // namespace tvvf_vo_c
