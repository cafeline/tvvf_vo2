#include "tvvf_vo_c/ros/tvvf_vo_node.hpp"
#include <cmath>
#include <algorithm>

namespace tvvf_vo_c
{
  namespace {
    constexpr double MIN_VECTOR_MAGNITUDE = 0.01;
    constexpr double VECTOR_FIELD_PUBLISH_INTERVAL_SEC = 0.5;
  }

  void TVVFVONode::publish_empty_visualization()
  {
    // PoseArray のみで、特にクリア処理は不要
  }

  geometry_msgs::msg::PoseArray TVVFVONode::build_vector_pose_array(
      const VectorField& field) const
  {
    geometry_msgs::msg::PoseArray pose_array;
    pose_array.header.frame_id = cached_params_.global_frame;
    pose_array.header.stamp = this->now();

    auto is_in_overlay = [&](const Position& pos) {
      for (const auto& overlay : field.overlays) {
        FieldRegion region = overlay.region.has_value()
            ? overlay.region.value()
            : FieldRegion(
                Position(overlay.origin.x, overlay.origin.y),
                Position(
                  overlay.origin.x +
                    static_cast<double>(overlay.width) * overlay.resolution,
                  overlay.origin.y +
                    static_cast<double>(overlay.height) * overlay.resolution));
        if (region.contains(pos)) {
          return true;
        }
      }
      return false;
    };

    auto append_samples = [&](const VectorField& vf, bool skip_overlay_region) {
      for (int y = 0; y < vf.height; ++y) {
        for (int x = 0; x < vf.width; ++x) {
          const Position world_pos = vf.gridToWorld(x, y);
          if (skip_overlay_region && is_in_overlay(world_pos)) {
            continue;
          }
          const auto combined_vector =
            compute_navigation_vector(vf.vectors[static_cast<size_t>(y)][static_cast<size_t>(x)],
                                      world_pos);
          if (!is_valid_position(world_pos) || !is_valid_vector(combined_vector)) {
            continue;
          }
          if (!should_visualize_vector(combined_vector)) {
            continue;
          }
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
    };

    // base field at its own resolution, skipping overlay regions
    append_samples(field, !field.overlays.empty());

    // overlay fields at their own resolutions
    for (const auto& overlay : field.overlays) {
      append_samples(overlay, false);
    }

    return pose_array;
  }

  void TVVFVONode::publish_combined_field_visualization(const VectorField& field)
  {
    if (!use_rviz_) {
      return;
    }
    try
    {
      const auto steady_now = steady_clock_.now();
      const double elapsed = (steady_now - last_vector_field_publish_time_).seconds();
      if (elapsed < VECTOR_FIELD_PUBLISH_INTERVAL_SEC) {
        return;
      }

      auto pose_array = build_vector_pose_array(field);

      if (vector_field_pose_pub_ && !pose_array.poses.empty()) {
        vector_field_pose_pub_->publish(pose_array);
        last_vector_field_publish_time_ = steady_now;
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

    double target_resolution = cost_map.resolution;
    for (const auto& overlay : cost_map.overlays) {
      if (overlay.isValid()) {
        target_resolution = std::min(target_resolution, overlay.resolution);
      }
    }
    target_resolution = std::max(target_resolution, 1e-6);

    const double size_x =
      static_cast<double>(cost_map.width) * cost_map.resolution;
    const double size_y =
      static_cast<double>(cost_map.height) * cost_map.resolution;
    grid.info.width = static_cast<uint32_t>(std::ceil(size_x / target_resolution));
    grid.info.height = static_cast<uint32_t>(std::ceil(size_y / target_resolution));
    grid.info.resolution = target_resolution;
    grid.info.origin.position.x = cost_map.origin.x;
    grid.info.origin.position.y = cost_map.origin.y;
    grid.info.origin.orientation.w = 1.0;

    grid.data.assign(static_cast<size_t>(grid.info.width) * grid.info.height, 0);

    const double max_clearance = std::max(cost_map_settings_.max_clearance, 1e-6);
    const double origin_x = grid.info.origin.position.x;
    const double origin_y = grid.info.origin.position.y;
    for (uint32_t row = 0; row < grid.info.height; ++row) {
      for (uint32_t col = 0; col < grid.info.width; ++col) {
        const double wx = origin_x + (static_cast<double>(col) + 0.5) * target_resolution;
        const double wy = origin_y + (static_cast<double>(row) + 0.5) * target_resolution;
        const Position p(wx, wy);

        double clearance = cost_map.clearanceAtWorld(p);
        if (!std::isfinite(clearance)) {
          clearance = 0.0;
        }
        clearance = std::clamp(clearance, 0.0, max_clearance);
        double normalized = 1.0 - (clearance / max_clearance);
        int cost = static_cast<int>(std::lround(normalized * 100.0));

        const double speed = cost_map.speedAtWorld(p);
        if (speed <= 0.0) {
          cost = 100;
        }

        cost = std::clamp(cost, 0, 100);
        grid.data[static_cast<size_t>(row) * grid.info.width + static_cast<size_t>(col)] =
          static_cast<int8_t>(cost);
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
