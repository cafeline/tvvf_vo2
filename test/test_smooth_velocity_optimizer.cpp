// test/test_smooth_velocity_optimizer.cpp

#include <gtest/gtest.h>
#include "tvvf_vo_c/core/smooth_velocity_optimizer.hpp"

using namespace tvvf_vo_c;

class SmoothVelocityOptimizerTest : public ::testing::Test {
protected:
  OptimizationOptions options;
  SmoothVelocityOptimizer optimizer;

  SmoothVelocityOptimizerTest() : optimizer(options) {}
};

TEST_F(SmoothVelocityOptimizerTest, FollowsDesiredWithoutObstacles) {
  options.goal_weight = 1.0;
  options.smooth_weight = 0.0;
  options.obstacle_weight = 0.0;
  options.max_linear_acceleration = 10.0;
  optimizer.setOptions(options);

  OptimizationState state;
  state.desired_velocity = Velocity(0.8, 0.0);
  state.previous_velocity = Velocity(0.0, 0.0);
  state.max_speed = 1.0;
  state.dt = 1.0;

  auto result = optimizer.computeOptimalVelocity(state);
  EXPECT_NEAR(result.vx, 0.8, 1e-3);
  EXPECT_NEAR(result.vy, 0.0, 1e-3);
}

TEST_F(SmoothVelocityOptimizerTest, PrefersPreviousWhenSmoothWeightHigh) {
  options.goal_weight = 1.0;
  options.smooth_weight = 10.0;
  optimizer.setOptions(options);

  OptimizationState state;
  state.desired_velocity = Velocity(1.0, 0.0);
  state.previous_velocity = Velocity(0.2, 0.0);
  state.max_speed = 1.0;

  auto result = optimizer.computeOptimalVelocity(state);
  EXPECT_LT(result.vx, 0.5);
  EXPECT_GT(result.vx, 0.2);
}

TEST_F(SmoothVelocityOptimizerTest, AvoidsStaticObstacleAhead) {
  OptimizationState state;
  state.desired_velocity = Velocity(0.6, 0.0);
  state.previous_velocity = Velocity(0.6, 0.0);
  state.robot_position = Position(0.0, 0.0);
  state.static_obstacles = {Position(0.5, 0.0)};
  state.max_speed = 1.0;

  auto result = optimizer.computeOptimalVelocity(state);
  EXPECT_NEAR(result.vx, state.previous_velocity.vx, 1e-6);
}

TEST_F(SmoothVelocityOptimizerTest, RespectsAccelerationLimit) {
  options.max_linear_acceleration = 0.2;
  optimizer.setOptions(options);

  OptimizationState state;
  state.desired_velocity = Velocity(1.0, 0.0);
  state.previous_velocity = Velocity(0.0, 0.0);
  state.max_speed = 1.0;
  state.dt = 0.1;

  auto result = optimizer.computeOptimalVelocity(state);
  EXPECT_NEAR(result.vx, options.max_linear_acceleration * state.dt, 1e-6);
}
