//
// smooth_velocity_optimizer.cpp
//

#include "tvvf_vo_c/core/smooth_velocity_optimizer.hpp"
#include <algorithm>
#include <cmath>

namespace tvvf_vo_c {

namespace {
constexpr double EPSILON = 1e-6;

std::array<double, 2> toArray(const Velocity& v) {
  return {v.vx, v.vy};
}

Velocity fromArray(const std::array<double, 2>& arr) {
  return Velocity(arr[0], arr[1]);
}

double magnitude(const std::array<double, 2>& arr) {
  return std::sqrt(arr[0] * arr[0] + arr[1] * arr[1]);
}

std::array<double, 2> clampMagnitude(const std::array<double, 2>& arr,
                                     double limit) {
  double norm = magnitude(arr);
  if (norm < EPSILON || norm <= limit) {
    return arr;
  }
  double scale = limit / norm;
  return {arr[0] * scale, arr[1] * scale};
}

}  // namespace

SmoothVelocityOptimizer::SmoothVelocityOptimizer(const OptimizationOptions& options)
    : options_(options) {}

Velocity SmoothVelocityOptimizer::clampVelocity(
    const Velocity& candidate,
    const Velocity& previous,
    double max_speed,
    double dt) const {
  auto array_candidate = toArray(candidate);
  auto array_prev = toArray(previous);

  double max_delta = options_.max_linear_acceleration * dt;
  std::array<double, 2> delta = {
      array_candidate[0] - array_prev[0],
      array_candidate[1] - array_prev[1]
  };
  delta = clampMagnitude(delta, max_delta);

  std::array<double, 2> limited = {
      array_prev[0] + delta[0],
      array_prev[1] + delta[1]
  };

  limited = clampMagnitude(limited, max_speed);
  return fromArray(limited);
}

Velocity SmoothVelocityOptimizer::computeOptimalVelocity(
    const OptimizationState& state) const {
  std::array<double, 2> desired = toArray(state.desired_velocity);
  std::array<double, 2> previous = toArray(state.previous_velocity);

  double weights_sum = 0.0;
  std::array<double, 2> numerator = {0.0, 0.0};

  if (options_.goal_weight > 0.0) {
    weights_sum += options_.goal_weight;
    numerator[0] += options_.goal_weight * desired[0];
    numerator[1] += options_.goal_weight * desired[1];
  }

  if (options_.smooth_weight > 0.0) {
    weights_sum += options_.smooth_weight;
    numerator[0] += options_.smooth_weight * previous[0];
    numerator[1] += options_.smooth_weight * previous[1];
  }


  Velocity blended;
  if (weights_sum <= EPSILON) {
    blended = state.desired_velocity;
  } else {
    blended = Velocity(numerator[0] / weights_sum, numerator[1] / weights_sum);
  }

  return clampVelocity(blended, state.previous_velocity, state.max_speed, state.dt);
}

}  // namespace tvvf_vo_c
