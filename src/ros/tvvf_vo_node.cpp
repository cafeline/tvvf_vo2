#include "tvvf_vo_c/ros/tvvf_vo_node.hpp"
#include <chrono>
#include <algorithm>

namespace tvvf_vo_c
{

  TVVFVONode::TVVFVONode() : Node("tvvf_vo_c_node")
  {
    // パラメータ設定
    setup_parameters();

    // 設定初期化
    config_ = create_config_from_parameters();
    base_frame_ = this->get_parameter("base_frame").as_string();
    cached_params_.base_frame = base_frame_;
    cached_params_.global_frame = this->get_parameter("global_frame").as_string();
    cached_params_.goal_tolerance = this->get_parameter("goal_tolerance").as_double();
    cached_params_.orientation_tolerance = this->get_parameter("orientation_tolerance").as_double();
    cached_params_.turning_linear_scale = this->get_parameter("turning_linear_scale").as_double();
    cached_params_.turning_angular_gain = this->get_parameter("turning_angular_gain").as_double();
    cached_params_.tracking_angular_gain = this->get_parameter("tracking_angular_gain").as_double();
    cached_params_.occupancy_clear_radius = this->get_parameter("occupancy_clear_radius").as_double();
    cached_params_.costmap_resolution = this->get_parameter("costmap_resolution").as_double();
    cached_params_.vector_field_path_width = this->get_parameter("vector_field_path_width").as_double();
    cached_params_.robot_radius = this->get_parameter("robot_radius").as_double();

    // グローバルフィールドジェネレータ初期化（動的障害物は常に考慮）
    global_field_generator_ = std::make_unique<GlobalFieldGenerator>();
    global_field_generator_->setCostMapSettings(cost_map_settings_);

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
    auto costmap_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
    costmap_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("tvvf_vo_costmap", costmap_qos);

    // サブスクライバー初期化
    auto map_qos = rclcpp::QoS(1).reliable().transient_local();
    map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
        "map", map_qos, std::bind(&TVVFVONode::map_callback, this, std::placeholders::_1));

    clicked_point_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
        "clicked_point", 10, std::bind(&TVVFVONode::clicked_point_callback, this, std::placeholders::_1));

    goal_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "goal_pose", 10, std::bind(&TVVFVONode::goal_pose_callback, this, std::placeholders::_1));

    const auto mask_topic = this->get_parameter("obstacle_mask_topic").as_string();
    auto mask_qos = rclcpp::QoS(1).reliable().transient_local();
    obstacle_mask_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
        mask_topic, mask_qos,
        std::bind(&TVVFVONode::obstacle_mask_callback, this, std::placeholders::_1));

    // タイマー初期化（制御ループレートはパラメータ化）
    double control_rate_hz = this->get_parameter("control_loop_rate").as_double();
    if (control_rate_hz <= 0.0) {
      RCLCPP_WARN(this->get_logger(),
                  "control_loop_rate <= 0 specified (%.3f). Using 20 Hz fallback.",
                  control_rate_hz);
      control_rate_hz = 20.0;
    }
    auto control_period = std::chrono::duration<double>(1.0 / control_rate_hz);
    control_timer_ = this->create_wall_timer(
        control_period, std::bind(&TVVFVONode::control_loop, this));

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
    this->declare_parameter("obstacle_mask_topic", "/obstacle_mask");
    this->declare_parameter("occupancy_clear_radius", 0.3);
    this->declare_parameter("orientation_tolerance", 0.2);
    this->declare_parameter("turning_linear_scale", 0.3);
    this->declare_parameter("turning_angular_gain", 2.0);
    this->declare_parameter("tracking_angular_gain", 1.0);
    this->declare_parameter("control_loop_rate", 20.0);

    // コストマップ関連
    this->declare_parameter("costmap_occupied_threshold", 50.0);
    this->declare_parameter("costmap_free_threshold", 10.0);
    this->declare_parameter("costmap_alpha", 1.0);
    this->declare_parameter("costmap_min_speed", 0.05);
    this->declare_parameter("costmap_max_speed", 1.0);
    this->declare_parameter("costmap_clearance_epsilon", 0.1);
    this->declare_parameter("costmap_max_clearance", 5.0);
    this->declare_parameter("costmap_resolution", 0.0);
    this->declare_parameter("optimizer_goal_weight", 1.0);
    this->declare_parameter("optimizer_smooth_weight", 4.0);
    this->declare_parameter("optimizer_max_linear_acceleration", 0.5);

  }

  TVVFVOConfig TVVFVONode::create_config_from_parameters()
  {
    TVVFVOConfig config;

    config.max_linear_velocity = this->get_parameter("max_linear_velocity").as_double();
    config.max_angular_velocity = this->get_parameter("max_angular_velocity").as_double();

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
    config.max_linear_acceleration =
        std::max(0.01, this->get_parameter("optimizer_max_linear_acceleration").as_double());

    return config;
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
    return cached_params_.vector_field_path_width;
  }

  OptimizationOptions TVVFVONode::build_optimizer_options() const
  {
    OptimizationOptions options;
    options.goal_weight = config_.goal_weight;
    options.smooth_weight = config_.smooth_weight;
    options.max_linear_acceleration = config_.max_linear_acceleration;
    return options;
  }

} // namespace tvvf_vo_c
