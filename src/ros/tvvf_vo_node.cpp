#include "tvvf_vo_c/ros/tvvf_vo_node.hpp"
#include <chrono>
#include <algorithm>

namespace tvvf_vo_c
{

  TVVFVONode::TVVFVONode() : Node("tvvf_vo_c_node"), field_update_pending_(false)
  {
    // パラメータ設定
    setup_parameters();

    // 設定初期化
    config_ = create_config_from_parameters();
    base_frame_ = this->get_parameter("base_frame").as_string();
    map_obstacles_dirty_ = true;

    // グローバルフィールドジェネレータ初期化
    global_field_generator_ = std::make_unique<GlobalFieldGenerator>();
    global_field_generator_->setDynamicRepulsionEnabled(enable_global_repulsion_);
    global_field_generator_->setCostMapSettings(cost_map_settings_);

    // 斥力計算器初期化
    repulsive_force_calculator_ = std::make_unique<RepulsiveForceCalculator>();
    repulsive_force_calculator_->setConfig(repulsive_config_);
    velocity_optimizer_ = std::make_unique<SmoothVelocityOptimizer>(build_optimizer_options());

    // TF2関連初期化
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // パブリッシャー初期化
    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel_raw", 10);
    auto pose_qos = rclcpp::QoS(10);
    pose_qos.best_effort();
    vector_field_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseArray>("tvvf_vo_vector_pose_array", pose_qos);
    planned_path_pub_ = this->create_publisher<nav_msgs::msg::Path>("planned_path", 10);
    goal_marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("goal_marker", 10);
    cmd_velocity_marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("cmd_vel_marker", 10);

    // サブスクライバー初期化
    auto map_qos = rclcpp::QoS(1).reliable().transient_local();
    map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
        "map", map_qos, std::bind(&TVVFVONode::map_callback, this, std::placeholders::_1));

    clicked_point_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
        "clicked_point", 10, std::bind(&TVVFVONode::clicked_point_callback, this, std::placeholders::_1));

    goal_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "goal_pose", 10, std::bind(&TVVFVONode::goal_pose_callback, this, std::placeholders::_1));

  // 障害物データをsubscribe（統合コールバック使用）
  obstacles_sub_ = this->create_subscription<visualization_msgs::msg::MarkerArray>(
      "obstacles", 10,
      std::bind(&TVVFVONode::obstacles_callback, this, std::placeholders::_1));

    // タイマー初期化（制御ループ：20Hz）
    control_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(50), std::bind(&TVVFVONode::control_loop, this));

  }

  void TVVFVONode::setup_parameters()
  {

    // ロボット仕様パラメータ
    this->declare_parameter("max_linear_velocity", 2.0);
    this->declare_parameter("max_angular_velocity", 2.0);
    this->declare_parameter("robot_radius", 0.25);

    // フレーム名
    this->declare_parameter("base_frame", "base_footprint");
    this->declare_parameter("global_frame", "map");

    // 制御関連
    this->declare_parameter("goal_tolerance", 0.1);
    this->declare_parameter("vector_field_path_width", 4.0);
    this->declare_parameter("vector_field_publish_interval", 0.2);

    // 斥力パラメータ
    this->declare_parameter("repulsive_strength", 1.0);
    this->declare_parameter("repulsive_influence_range", 2.0);
    this->declare_parameter("enable_global_repulsion", true);
    this->declare_parameter("enable_repulsive_field", false);
    this->declare_parameter("costmap_occupied_threshold", 50.0);
    this->declare_parameter("costmap_free_threshold", 10.0);
    this->declare_parameter("costmap_alpha", 1.0);
    this->declare_parameter("costmap_min_speed", 0.05);
    this->declare_parameter("costmap_max_speed", 1.0);
    this->declare_parameter("costmap_clearance_epsilon", 0.1);
    this->declare_parameter("costmap_max_clearance", 5.0);
    this->declare_parameter("optimizer_goal_weight", 1.0);
    this->declare_parameter("optimizer_smooth_weight", 4.0);
    this->declare_parameter("optimizer_obstacle_weight", 2.0);
    this->declare_parameter("optimizer_obstacle_influence_range", 1.5);
    this->declare_parameter("optimizer_obstacle_safe_distance", 0.4);
    this->declare_parameter("optimizer_max_linear_acceleration", 0.5);

  }

  TVVFVOConfig TVVFVONode::create_config_from_parameters()
  {
    TVVFVOConfig config;

    config.max_linear_velocity = this->get_parameter("max_linear_velocity").as_double();
    config.max_angular_velocity = this->get_parameter("max_angular_velocity").as_double();

    // 斥力設定の読み込み
    repulsive_config_.repulsive_strength = this->get_parameter("repulsive_strength").as_double();
    repulsive_config_.influence_range = this->get_parameter("repulsive_influence_range").as_double();
    enable_global_repulsion_ = this->get_parameter("enable_global_repulsion").as_bool();
    enable_repulsive_field_ = this->get_parameter("enable_repulsive_field").as_bool();

    vector_field_publish_interval_ = std::max(0.0, this->get_parameter("vector_field_publish_interval").as_double());
    RCLCPP_INFO(this->get_logger(), "Vector field publish interval set to %.3f [s]", vector_field_publish_interval_);

    cost_map_settings_.occupied_threshold =
        std::clamp(this->get_parameter("costmap_occupied_threshold").as_double(), 0.0, 100.0);
    cost_map_settings_.free_threshold =
        std::clamp(this->get_parameter("costmap_free_threshold").as_double(), 0.0, 100.0);
    cost_map_settings_.free_threshold = std::min(cost_map_settings_.free_threshold, cost_map_settings_.occupied_threshold);
    cost_map_settings_.alpha = std::max(0.0, this->get_parameter("costmap_alpha").as_double());
    cost_map_settings_.min_speed = std::max(0.0, this->get_parameter("costmap_min_speed").as_double());
    cost_map_settings_.max_speed = std::max(cost_map_settings_.min_speed,
        this->get_parameter("costmap_max_speed").as_double());
    cost_map_settings_.clearance_epsilon =
        std::max(0.01, this->get_parameter("costmap_clearance_epsilon").as_double());
    cost_map_settings_.max_clearance =
        std::max(cost_map_settings_.clearance_epsilon,
                 this->get_parameter("costmap_max_clearance").as_double());

    config.goal_weight = std::max(0.0, this->get_parameter("optimizer_goal_weight").as_double());
    config.smooth_weight = std::max(0.0, this->get_parameter("optimizer_smooth_weight").as_double());
    config.obstacle_weight = std::max(0.0, this->get_parameter("optimizer_obstacle_weight").as_double());
    config.obstacle_influence_range =
        std::max(0.1, this->get_parameter("optimizer_obstacle_influence_range").as_double());
    config.obstacle_safe_distance =
        std::max(0.05, this->get_parameter("optimizer_obstacle_safe_distance").as_double());
    config.max_linear_acceleration =
        std::max(0.01, this->get_parameter("optimizer_max_linear_acceleration").as_double());

    return config;
  }

  void TVVFVONode::request_static_field_update()
  {
    field_update_pending_ = true;
  }

  bool TVVFVONode::try_recompute_static_field()
  {
    if (!field_update_pending_) {
      return true;
    }

    if (!goal_.has_value()) {
      field_update_pending_ = false;
      return true;
    }

    if (!current_map_.has_value() || !global_field_generator_) {
      return false;
    }

    if (!robot_state_.has_value()) {
      // ロボット姿勢が取得できなくても全域で計算する
      update_robot_state();
    }

    auto region = robot_state_.has_value() ? build_planning_region() : std::optional<FieldRegion>{std::nullopt};

    try {
      global_field_generator_->precomputeStaticField(
          current_map_.value(), goal_->position, region);
      field_update_pending_ = false;
      return true;
    } catch (const std::exception& e) {
      RCLCPP_ERROR(this->get_logger(), "Failed to compute static field: %s", e.what());
      return false;
    }
  }

  std::optional<FieldRegion> TVVFVONode::build_planning_region() const
  {
    if (!goal_.has_value() || !robot_state_.has_value()) {
      return std::nullopt;
    }

    const double path_width = std::max(get_path_width_parameter(), 0.0);
    return create_path_region(
        robot_state_->position,
        goal_->position,
        path_width,
        REGION_MARGIN_METERS);
  }

  double TVVFVONode::get_path_width_parameter() const
  {
    if (!this->has_parameter("vector_field_path_width")) {
      return 0.0;
    }
    return this->get_parameter("vector_field_path_width").as_double();
  }

  OptimizationOptions TVVFVONode::build_optimizer_options() const
  {
    OptimizationOptions options;
    options.goal_weight = config_.goal_weight;
    options.smooth_weight = config_.smooth_weight;
    options.obstacle_weight = config_.obstacle_weight;
    options.obstacle_influence_range = config_.obstacle_influence_range;
    options.repulsive_strength = repulsive_config_.repulsive_strength;
    options.repulsive_influence_range = repulsive_config_.influence_range;
    options.obstacle_safe_distance = config_.obstacle_safe_distance;
    options.max_linear_acceleration = config_.max_linear_acceleration;
    return options;
  }

} // namespace tvvf_vo_c
