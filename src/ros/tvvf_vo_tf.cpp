#include "tvvf_vo_c/ros/tvvf_vo_node.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

namespace tvvf_vo_c
{

  std::optional<RobotState> TVVFVONode::get_robot_pose_from_tf()
  {
    try
    {
      const std::string& base_frame = cached_params_.base_frame;
      const std::string& global_frame = cached_params_.global_frame;

      // TF取得
      geometry_msgs::msg::TransformStamped transform_stamped;
      transform_stamped = tf_buffer_->lookupTransform(
          global_frame, base_frame, tf2::TimePointZero, tf2::durationFromSec(0.1));

      // 位置の取得
      Position position(transform_stamped.transform.translation.x,
                        transform_stamped.transform.translation.y);

      // 姿勢角の取得
      tf2::Quaternion q(
          transform_stamped.transform.rotation.x,
          transform_stamped.transform.rotation.y,
          transform_stamped.transform.rotation.z,
          transform_stamped.transform.rotation.w);
      tf2::Matrix3x3 m(q);
      double roll, pitch, yaw;
      m.getRPY(roll, pitch, yaw);

      // 速度計算（簡易版）
      Velocity velocity(0.0, 0.0);

      // ロボット状態作成
      RobotState robot_state(
          position, velocity, yaw,
          config_.max_linear_velocity,
          1.0,  // max_acceleration固定値
          cached_params_.robot_radius);

      return robot_state;
    }
    catch (const tf2::TransformException &ex)
    {
      return std::nullopt;
    }
  }

} // namespace tvvf_vo_c
