//
// smooth_velocity_optimizer.hpp
//

#ifndef TVVF_VO_C_CORE_SMOOTH_VELOCITY_OPTIMIZER_HPP_
#define TVVF_VO_C_CORE_SMOOTH_VELOCITY_OPTIMIZER_HPP_

#include "tvvf_vo_c/core/types.hpp"
#include <vector>

namespace tvvf_vo_c {

struct OptimizationOptions {
  double goal_weight{1.0};
  double smooth_weight{4.0};
  double max_linear_acceleration{0.5};
};

struct OptimizationState {
  Velocity desired_velocity;
  Velocity previous_velocity;
  Position robot_position;
  std::vector<Position> static_obstacles;
  std::vector<DynamicObstacle> dynamic_obstacles;
  double max_speed{0.5};
  double dt{0.05};
};

class SmoothVelocityOptimizer {
public:
  explicit SmoothVelocityOptimizer(const OptimizationOptions& options = OptimizationOptions());

  Velocity computeOptimalVelocity(const OptimizationState& state) const;

  void setOptions(const OptimizationOptions& options) { options_ = options; }

private:
  Velocity clampVelocity(const Velocity& candidate,
                         const Velocity& previous,
                         double max_speed,
                         double dt) const;

  OptimizationOptions options_;
};

}  // namespace tvvf_vo_c

#endif  // TVVF_VO_C_CORE_SMOOTH_VELOCITY_OPTIMIZER_HPP_
