#ifndef TVVF_VO_C_CORE_TYPES_HPP_
#define TVVF_VO_C_CORE_TYPES_HPP_

#include <vector>
#include <optional>
#include <cmath>
#include <memory>

namespace tvvf_vo_c {

/**
 * @brief 2D位置クラス
 */
struct Position {
    double x, y;
    
    Position() : x(0.0), y(0.0) {}
    Position(double x, double y) : x(x), y(y) {}
    
    double distance_to(const Position& other) const;
    Position operator+(const Position& other) const;
    Position operator-(const Position& other) const;
    Position operator*(double scalar) const;
    bool operator==(const Position& other) const;
};

/**
 * @brief 2D速度クラス
 */
struct Velocity {
    double vx, vy;
    
    Velocity() : vx(0.0), vy(0.0) {}
    Velocity(double vx, double vy) : vx(vx), vy(vy) {}
    
    double magnitude() const;
    Velocity normalize() const;
    Velocity operator+(const Velocity& other) const;
    Velocity operator-(const Velocity& other) const;
    Velocity operator*(double scalar) const;
};

/**
 * @brief ロボット状態クラス
 */
struct RobotState {
    Position position;
    Velocity velocity;
    double orientation;          // [rad]
    double max_velocity;         // [m/s]
    double max_acceleration;     // [m/s²]
    double radius;               // [m]
    
    RobotState() 
        : orientation(0.0), max_velocity(2.0), max_acceleration(1.0), radius(0.3) {}
    
    RobotState(const Position& pos, const Velocity& vel, double orient,
               double max_vel = 2.0, double max_acc = 1.0, double r = 0.3)
        : position(pos), velocity(vel), orientation(orient), 
          max_velocity(max_vel), max_acceleration(max_acc), radius(r) {}
};

/**
 * @brief 動的障害物クラス
 */
struct DynamicObstacle {
    int id;
    Position position;
    Velocity velocity;
    double radius;
    double prediction_horizon;   // [s]
    double uncertainty;          // [m]
    
    DynamicObstacle() 
        : id(0), radius(0.1), prediction_horizon(3.0), uncertainty(0.1) {}
    
    DynamicObstacle(int id, const Position& pos, const Velocity& vel, double r,
                     double pred_horizon = 3.0, double uncert = 0.1)
        : id(id), position(pos), velocity(vel), radius(r), 
          prediction_horizon(pred_horizon), uncertainty(uncert) {}
    
    // GlobalFieldGeneratorとの互換性のためのコンストラクタ
    DynamicObstacle(const Position& pos, const Velocity& vel, double r)
        : id(0), position(pos), velocity(vel), radius(r), 
          prediction_horizon(3.0), uncertainty(0.1) {}
    
    Position predict_position(double time_delta) const;
};

/**
 * @brief 目標クラス
 */
struct Goal {
    Position position;
    double tolerance;            // [m]
    
    Goal() : tolerance(0.1) {}
    Goal(const Position& pos, double tol = 0.1) : position(pos), tolerance(tol) {}
};

/**
 * @brief 制御出力クラス
 */
struct ControlOutput {
    Velocity velocity_command;
    double angular_velocity;     // [rad/s]
    double execution_time;       // [s]
    double safety_margin;        // [m]
    
    ControlOutput() 
        : angular_velocity(0.0), execution_time(0.05), safety_margin(0.0) {}
    
    ControlOutput(const Velocity& vel_cmd, double ang_vel = 0.0, 
                  double exec_time = 0.05, double safety = 0.0)
        : velocity_command(vel_cmd), angular_velocity(ang_vel), 
          execution_time(exec_time), safety_margin(safety) {}
};

/**
 * @brief TVVF-VO統合システム設定
 */
struct TVVFVOConfig {
    // ロボット仕様パラメータ
    double max_linear_velocity;
    double max_angular_velocity;
    double goal_weight;
    double smooth_weight;
    double obstacle_weight;
    double obstacle_influence_range;
    double obstacle_safe_distance;
    double max_linear_acceleration;
    
    TVVFVOConfig() 
        : max_linear_velocity(2.0),
          max_angular_velocity(2.0),
          goal_weight(1.0),
          smooth_weight(4.0),
          obstacle_weight(2.0),
          obstacle_influence_range(1.0),
          obstacle_safe_distance(0.4),
          max_linear_acceleration(0.5) {}
};

} // namespace tvvf_vo_c

#endif // TVVF_VO_C_CORE_TYPES_HPP_
