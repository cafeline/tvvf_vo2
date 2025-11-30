#include "tvvf_vo_c/ros/tvvf_vo_node.hpp"
#include <cmath>
#include <algorithm>

namespace tvvf_vo_c
{
  namespace {
    constexpr double MIN_VECTOR_MAGNITUDE = 0.01;
  }

  void TVVFVONode::publish_empty_visualization()
  {
    // PoseArray のみで、特にクリア処理は不要
  }

  void TVVFVONode::publish_combined_field_visualization(const VectorField& field)
  {
    try
    {
      geometry_msgs::msg::PoseArray pose_array;
      const std::string global_frame = this->get_parameter("global_frame").as_string();

      pose_array.header.frame_id = global_frame;
      pose_array.header.stamp = this->get_clock()->now();

      // ベクトル場と同じ解像度ですべてのセルを追加
      for (int y = 0; y < field.height; ++y)
      {
        for (int x = 0; x < field.width; ++x)
        {
          // ワールド座標に変換
          const Position world_pos = field.gridToWorld(x, y);

          // 合成ベクトルを計算（制御と同じ計算を利用）
          const auto combined_vector = compute_navigation_vector(field.vectors[y][x], world_pos);

          if (!is_valid_position(world_pos) || !is_valid_vector(combined_vector)) {
            RCLCPP_WARN_THROTTLE(
                this->get_logger(), *this->get_clock(), 2000,
                "Skipping invalid vector field sample (pos=(%.3f, %.3f), vec=(%.3f, %.3f))",
                world_pos.x, world_pos.y, combined_vector[0], combined_vector[1]);
            continue;
          }

          // ベクトルが小さすぎる場合はスキップ
          if (!should_visualize_vector(combined_vector)) {
            continue;
          }

          // PoseArrayへ追加（RVizで軽量表示用）
          geometry_msgs::msg::Pose pose;
          pose.position.x = world_pos.x;
          pose.position.y = world_pos.y;
          pose.position.z = 0.0;
          const double yaw = std::atan2(combined_vector[1], combined_vector[0]);
          tf2::Quaternion q;
          q.setRPY(0.0, 0.0, yaw);
          pose.orientation = tf2::toMsg(q);
          pose_array.poses.push_back(pose);
        }
      }

      if (vector_field_pose_pub_ && !pose_array.poses.empty()) {
        vector_field_pose_pub_->publish(pose_array);
      }
    }
    catch (const std::exception &e)
    {
      RCLCPP_ERROR(this->get_logger(), "Combined field visualization error: %s", e.what());
    }
  }

  std::array<double, 2> TVVFVONode::compute_navigation_vector(
      const std::array<double, 2>& base_vector,
      const Position& world_pos) const
  {
    // 元のベクトル場の値
    std::array<double, 2> combined = base_vector;

    // 斥力を追加（必要な場合のみ）
    if (enable_repulsive_field_ && repulsive_force_calculator_) {
      if (!static_obstacle_hulls_cache_.empty()) {
        const auto repulsive_force = repulsive_force_calculator_->calculateTotalForceFromHulls(
            world_pos, static_obstacle_hulls_cache_);
        combined[0] += repulsive_force.x;
        combined[1] += repulsive_force.y;
      }
      if (!static_obstacle_positions_cache_.empty()) {
        const auto repulsive_force = repulsive_force_calculator_->calculateTotalForceFromPositions(
            world_pos, static_obstacle_positions_cache_);
        combined[0] += repulsive_force.x;
        combined[1] += repulsive_force.y;
      }
    }

    return combined;
  }

  // ヘルパー関数: ベクトルを可視化すべきか判定
  bool TVVFVONode::should_visualize_vector(const std::array<double, 2>& vector) const
  {
    return std::abs(vector[0]) > MIN_VECTOR_MAGNITUDE ||
           std::abs(vector[1]) > MIN_VECTOR_MAGNITUDE;
  }

  bool TVVFVONode::is_valid_position(const Position& position) const
  {
    return std::isfinite(position.x) && std::isfinite(position.y);
  }

  bool TVVFVONode::is_valid_vector(const std::array<double, 2>& vector) const
  {
    return std::isfinite(vector[0]) && std::isfinite(vector[1]);
  }

} // namespace tvvf_vo_c
