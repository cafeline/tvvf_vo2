#ifndef TVVF_VO_C_ROS_TVVF_VO_NODE_HPP_
#define TVVF_VO_C_ROS_TVVF_VO_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/path.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "tvvf_vo_c/core/types.hpp"
#include "tvvf_vo_c/core/global_field_generator.hpp"
#include "tvvf_vo_c/core/smooth_velocity_optimizer.hpp"
#include "tvvf_vo_c/core/region_utils.hpp"
#include <memory>
#include <vector>
#include <optional>
#include <string>
#include <deque>
#include <numeric>

namespace tvvf_vo_c {

// 可視化パラメータの定数
/**
 * @brief TVVF-VO ROS2ナビゲーションノード
 */
class TVVFVONode : public rclcpp::Node {
public:
    /**
     * @brief コンストラクタ
     */
    TVVFVONode();

private:
    // コア機能
    std::unique_ptr<GlobalFieldGenerator> global_field_generator_;
    std::unique_ptr<SmoothVelocityOptimizer> velocity_optimizer_;
    TVVFVOConfig config_;
    CostMapSettings cost_map_settings_;
    struct CachedParams {
        std::string base_frame;
        std::string global_frame;
        double goal_tolerance{0.0};
        double orientation_tolerance{0.0};
        double turning_linear_scale{0.0};
        double turning_angular_gain{0.0};
        double tracking_angular_gain{0.0};
        double occupancy_clear_radius{0.0};
        double costmap_resolution{0.0};
        double vector_field_path_width{0.0};
        double robot_radius{0.0};
    } cached_params_;
    std::optional<VectorField> latest_field_;
    std::optional<Velocity> previous_velocity_command_;
    std::string base_frame_;
    int cmd_velocity_marker_seq_{0};
    std::deque<double> timing_total_ms_window_;

    // 状態変数
    std::optional<RobotState> robot_state_;
    std::optional<Goal> goal_;
    std::optional<nav_msgs::msg::OccupancyGrid> obstacle_mask_;
    std::optional<nav_msgs::msg::OccupancyGrid> current_map_;
    rclcpp::Time last_vector_field_publish_time_;
    int last_vector_field_marker_count_{0};

    // TF2関連
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    // パブリッシャー
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr vector_field_pose_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr planned_path_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr goal_marker_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr cmd_velocity_marker_pub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_pub_;

    // サブスクライバー
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr clicked_point_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pose_sub_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr obstacle_mask_sub_;

    // タイマー
    rclcpp::TimerBase::SharedPtr control_timer_;


public:

    /**
     * @brief パラメータ設定
     */
    void setup_parameters();

    /**
     * @brief ROS2パラメータからTVVFVOConfigを作成
     * @return TVVF-VO設定
     */
    TVVFVOConfig create_config_from_parameters();

    /**
     * @brief TFからロボット位置を取得
     * @return ロボット状態（失敗時はstd::nullopt）
     */
    std::optional<RobotState> get_robot_pose_from_tf();

    /**
     * @brief クリックされたポイントをゴールとして設定
     * @param msg PointStampedメッセージ
     */
    void clicked_point_callback(const geometry_msgs::msg::PointStamped::SharedPtr msg);

    /**
     * @brief ゴールポーズをゴールとして設定
     * @param msg PoseStampedメッセージ
     */
    void goal_pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);

    /**
     * @brief 地図データコールバック
     * @param msg OccupancyGridメッセージ
     */
    void map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);

    /**
     * @brief 動的障害物マスクコールバック
     * @param msg OccupancyGridメッセージ
     */
    void obstacle_mask_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
    void refresh_map_obstacle_cache(const Position& robot_pos);

    // テスト用
    void debug_set_map(const nav_msgs::msg::OccupancyGrid & map) { current_map_ = map; }
    void debug_set_obstacle_mask(const nav_msgs::msg::OccupancyGrid & mask) { obstacle_mask_ = mask; }
    std::optional<nav_msgs::msg::OccupancyGrid> debug_build_combined_map() const { return build_combined_map(); }
    nav_msgs::msg::OccupancyGrid debug_build_costmap_grid(
        const CostMapResult& cost_map,
        const VectorField& field,
        const std::string& frame_id) const;
    void debug_set_robot_position(const Position & pos) {
      robot_state_ = RobotState(pos, Velocity(0.0, 0.0), 0.0, 0.0, 0.0, 0.0);
    }

    /**
     * @brief メイン制御ループ
     */
    void control_loop();

    /**
     * @brief 制御コマンドの配信（差動二輪用）
     * @param control_output 制御出力
     */
    void publish_control_command(const ControlOutput& control_output);

    /**
     * @brief ベクトル場からの速度指令を差動二輪用に変換
     * @param desired_vx 目標x方向速度
     * @param desired_vy 目標y方向速度
     * @param current_orientation 現在の姿勢角
     * @return 変換された（線形速度、角速度）
     */
    std::pair<double, double> convert_to_differential_drive(
        double desired_vx, double desired_vy, double current_orientation);

    /**
     * @brief 停止コマンドの配信
     */
    void publish_stop_command();

    /**
     * @brief 空の可視化マーカーを送信（クリア用）
     */
    void publish_empty_visualization();

    /**
     * @brief 合成ベクトル場の可視化
     * @param field 元のベクトル場
     */
    void publish_combined_field_visualization(const VectorField& field);

private:
    static constexpr double REGION_MARGIN_METERS = 5.0;

    // 制御ループヘルパー関数
    bool update_robot_state();
    bool has_valid_goal() const;
    bool is_goal_reached() const;
    void handle_goal_reached();
    ControlOutput compute_control_output();
    void update_visualization();
    void publish_planned_path();
    std::optional<FieldRegion> build_planning_region() const;
    double get_path_width_parameter() const;
    OptimizationOptions build_optimizer_options() const;
    void publish_command_marker(const geometry_msgs::msg::Twist& cmd_msg);
    std::optional<nav_msgs::msg::OccupancyGrid> build_combined_map() const;

private:
    friend class ObstaclesUnifiedTest;

    // 可視化ヘルパー関数
    std::array<double, 2> calculate_combined_vector(
        const std::array<double, 2>& original_vector,
        const Position& world_pos) const;
    std::array<double, 2> compute_navigation_vector(
        const std::array<double, 2>& base_vector,
        const Position& world_pos) const;
    nav_msgs::msg::OccupancyGrid build_costmap_grid(
        const CostMapResult& cost_map,
        const VectorField& field,
        const std::string& frame_id) const;
    nav_msgs::msg::OccupancyGrid resample_occupancy_grid(
        const nav_msgs::msg::OccupancyGrid& src,
        double new_resolution) const;
    void publish_costmap_visualization(
        const CostMapResult& cost_map,
        const VectorField& field,
        const std::string& frame_id);

    bool should_visualize_vector(const std::array<double, 2>& vector) const;
    bool is_valid_position(const Position& position) const;
    bool is_valid_vector(const std::array<double, 2>& vector) const;

};

} // namespace tvvf_vo_c

#endif // TVVF_VO_C_ROS_TVVF_VO_NODE_HPP_
