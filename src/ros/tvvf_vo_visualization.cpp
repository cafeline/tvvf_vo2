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
      const std::string global_frame = cached_params_.global_frame;

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
    (void)world_pos;
    return base_vector;
  }

  nav_msgs::msg::OccupancyGrid TVVFVONode::build_costmap_grid(
      const CostMapResult& cost_map,
      const VectorField& field,
      const std::string& frame_id) const
  {
    nav_msgs::msg::OccupancyGrid grid;

    if (!cost_map.isValid() || field.width <= 0 || field.height <= 0) {
      return grid;
    }

    grid.header.frame_id = frame_id.empty()
        ? cached_params_.global_frame
        : frame_id;
    grid.header.stamp = this->now();

    grid.info.width = cost_map.width;
    grid.info.height = cost_map.height;
    grid.info.resolution = cost_map.resolution;
    grid.info.origin.position.x = field.origin.x;
    grid.info.origin.position.y = field.origin.y;
    grid.info.origin.orientation.w = 1.0;

    grid.data.assign(static_cast<size_t>(grid.info.width) * grid.info.height, 0);

    const double max_clearance = std::max(cost_map_settings_.max_clearance, 1e-6);
    for (uint32_t y = 0; y < cost_map.height; ++y) {
      for (uint32_t x = 0; x < cost_map.width; ++x) {
        const size_t idx = static_cast<size_t>(y) * cost_map.width + x;
        double clearance = cost_map.clearance_layer[idx];
        if (!std::isfinite(clearance)) {
          clearance = 0.0;
        }
        clearance = std::clamp(clearance, 0.0, max_clearance);
        double normalized = 1.0 - (clearance / max_clearance);
        int cost = static_cast<int>(std::lround(normalized * 100.0));
        if (idx < cost_map.speed_layer.size() && cost_map.speed_layer[idx] <= 0.0) {
          cost = 100;
        }
        cost = std::clamp(cost, 0, 100);
        grid.data[idx] = static_cast<int8_t>(cost);
      }
    }

    return grid;
  }

  void TVVFVONode::publish_costmap_visualization(
      const CostMapResult& cost_map,
      const VectorField& field,
      const std::string& frame_id)
  {
    if (!costmap_pub_) {
      return;
    }
    auto msg = build_costmap_grid(cost_map, field, frame_id);
    if (msg.data.empty()) {
      return;
    }
    costmap_pub_->publish(msg);
  }

  nav_msgs::msg::OccupancyGrid TVVFVONode::debug_build_costmap_grid(
      const CostMapResult& cost_map,
      const VectorField& field,
      const std::string& frame_id) const
  {
    return build_costmap_grid(cost_map, field, frame_id);
  }

  nav_msgs::msg::OccupancyGrid TVVFVONode::resample_occupancy_grid(
      const nav_msgs::msg::OccupancyGrid& src,
      double new_resolution) const
  {
    nav_msgs::msg::OccupancyGrid out = src;
    if (new_resolution <= 0.0 || std::abs(new_resolution - src.info.resolution) < 1e-9) {
      return out;
    }

    const double size_x = static_cast<double>(src.info.width) * src.info.resolution;
    const double size_y = static_cast<double>(src.info.height) * src.info.resolution;
    const uint32_t new_width = static_cast<uint32_t>(std::ceil(size_x / new_resolution));
    const uint32_t new_height = static_cast<uint32_t>(std::ceil(size_y / new_resolution));

    out.info.resolution = new_resolution;
    out.info.width = new_width;
    out.info.height = new_height;
    out.data.assign(static_cast<size_t>(new_width) * new_height, -1);

    const double origin_x = src.info.origin.position.x;
    const double origin_y = src.info.origin.position.y;
    const double src_res = src.info.resolution;

    for (uint32_t row = 0; row < new_height; ++row) {
      for (uint32_t col = 0; col < new_width; ++col) {
        const double wx = origin_x + (static_cast<double>(col) + 0.5) * new_resolution;
        const double wy = origin_y + (static_cast<double>(row) + 0.5) * new_resolution;
        const int src_col = static_cast<int>(std::floor((wx - origin_x) / src_res));
        const int src_row = static_cast<int>(std::floor((wy - origin_y) / src_res));
        if (src_col < 0 || src_row < 0 ||
            src_col >= static_cast<int>(src.info.width) ||
            src_row >= static_cast<int>(src.info.height)) {
          continue;
        }
        const size_t src_idx =
          static_cast<size_t>(src_row) * src.info.width + static_cast<size_t>(src_col);
        out.data[static_cast<size_t>(row) * new_width + static_cast<size_t>(col)] =
          src.data[src_idx];
      }
    }
    return out;
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
