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

    // グローバルフィールドジェネレータ初期化
    global_field_generator_ = std::make_unique<GlobalFieldGenerator>();
    global_field_generator_->setDynamicRepulsionEnabled(enable_global_repulsion_);

    // 斥力計算器初期化
    repulsive_force_calculator_ = std::make_unique<RepulsiveForceCalculator>();
    repulsive_force_calculator_->setConfig(repulsive_config_);

    // TF2関連初期化
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // パブリッシャー初期化
    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel_raw", 10);
    vector_field_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("tvvf_vo_vector_field", 10);
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
    vector_field_publish_interval_ = std::max(0.0, this->get_parameter("vector_field_publish_interval").as_double());
    RCLCPP_INFO(this->get_logger(), "Vector field publish interval set to %.3f [s]", vector_field_publish_interval_);

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
