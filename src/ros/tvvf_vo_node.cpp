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
    map_obstacles_dirty_ = true;

    // グローバルフィールドジェネレータ初期化
    global_field_generator_ = std::make_unique<GlobalFieldGenerator>();
    global_field_generator_->setDynamicRepulsionEnabled(enable_global_repulsion_);
    global_field_generator_->setCostMapSettings(cost_map_settings_);

    // 斥力計算器初期化
    repulsive_force_calculator_ = std::make_unique<RepulsiveForceCalculator>();
    repulsive_force_calculator_->setConfig(repulsive_config_);

    // TF2関連初期化
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // パブリッシャー初期化
    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel_raw", 10);
    vector_field_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("tvvf_vo_vector_field", 10);
    map_obstacles_marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("map_static_obstacles_markers", 10);
    planned_path_pub_ = this->create_publisher<nav_msgs::msg::Path>("planned_path", 10);
    goal_marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("goal_marker", 10);

    // サブスクライバー初期化
    auto map_qos = rclcpp::QoS(1).reliable().transient_local();
    map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
        "map", map_qos, std::bind(&TVVFVONode::map_callback, this, std::placeholders::_1));

    clicked_point_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
        "clicked_point", 10, std::bind(&TVVFVONode::clicked_point_callback, this, std::placeholders::_1));

    goal_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "goal_pose", 10, std::bind(&TVVFVONode::goal_pose_callback, this, std::placeholders::_1));

    // 障害物データをsubscribe（統合コールバック使用）
    dynamic_obstacles_sub_ = this->create_subscription<visualization_msgs::msg::MarkerArray>(
        "dynamic_obstacles", 10,
        [this](const visualization_msgs::msg::MarkerArray::SharedPtr msg) { obstacles_callback(msg, true); });

    // 静的障害物データをsubscribe
    static_obstacles_sub_ = this->create_subscription<visualization_msgs::msg::MarkerArray>(
        "static_obstacles", 10,
        std::bind(&TVVFVONode::static_obstacles_callback, this, std::placeholders::_1));

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
    this->declare_parameter("map_repulsion_enabled", false);
    this->declare_parameter("map_repulsion_max_distance", 6.0);
    this->declare_parameter("map_repulsion_sampling_step", 0.5);
    this->declare_parameter("map_repulsion_recompute_distance", 1.0);
    this->declare_parameter("map_repulsion_occupancy_threshold", 50);
    this->declare_parameter("publish_map_obstacle_markers", true);
    this->declare_parameter("costmap_occupied_threshold", 50.0);
    this->declare_parameter("costmap_free_threshold", 10.0);
    this->declare_parameter("costmap_alpha", 1.0);
    this->declare_parameter("costmap_min_speed", 0.05);
    this->declare_parameter("costmap_max_speed", 1.0);
    this->declare_parameter("costmap_clearance_epsilon", 0.1);
    this->declare_parameter("costmap_max_clearance", 5.0);

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
    enable_map_repulsion_ = this->get_parameter("map_repulsion_enabled").as_bool();

    map_repulsion_settings_.max_distance = this->get_parameter("map_repulsion_max_distance").as_double();
    map_repulsion_settings_.sampling_step = this->get_parameter("map_repulsion_sampling_step").as_double();
    if (map_repulsion_settings_.sampling_step <= 0.0) {
      map_repulsion_settings_.sampling_step = 0.5;
    }
    const int occupancy_threshold =
        this->get_parameter("map_repulsion_occupancy_threshold").as_int();
    map_repulsion_settings_.occupancy_threshold = static_cast<int8_t>(
        std::clamp(occupancy_threshold, 0, 100));

    map_repulsion_recompute_distance_ =
        std::max(0.1, this->get_parameter("map_repulsion_recompute_distance").as_double());
    map_repulsion_recompute_distance_sq_ =
        map_repulsion_recompute_distance_ * map_repulsion_recompute_distance_;
    publish_map_obstacle_markers_ = this->get_parameter("publish_map_obstacle_markers").as_bool();
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
      if (!update_robot_state()) {
        return false;
      }
    }

    auto region = build_planning_region();

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

} // namespace tvvf_vo_c
