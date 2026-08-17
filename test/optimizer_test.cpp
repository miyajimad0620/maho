#include "maho/optimizer.hpp"

#include <cassert>
#include <cmath>
#include <vector>

namespace {

constexpr double kTolerance = 1e-9;

OptimizerParams MakeParams() {
  return {
      1.0,             // obstacle_cost_coefficient
      1.0,             // obstacle_influence_distance
      0.2,             // robot_radius
      1.0,             // goal_position_cost_coefficient
      1.0,             // goal_angle_cost_coefficient
      0.1,             // learning_rate
      1e-4,            // finite_difference_step
      20,              // max_iterations
      1.0,             // dt
      {0.1, 0.2, 0.3}  // max_velocity_change
  };
}

void TestEmptyPath() {
  const Optimizer optimizer(MakeParams());
  assert(optimizer.optimize({}, {}, {1.0, 2.0, 3.0}).empty());
}

void TestMovesTerminalNodeTowardGoal() {
  OptimizerParams params = MakeParams();
  params.max_velocity_change = {10.0, 10.0, 10.0};
  const Optimizer optimizer(params);
  const std::vector<Node> nodes{
      {{0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}},
      {{0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}},
  };
  const Pose2D goal{2.0, -1.0, 0.5};

  const std::vector<Node> optimized = optimizer.optimize(nodes, {}, goal);

  assert(std::hypot(optimized.back().pose.x - goal.x,
                    optimized.back().pose.y - goal.y) <
         std::hypot(nodes.back().pose.x - goal.x,
                    nodes.back().pose.y - goal.y));
  assert(std::abs(optimized.back().pose.theta - goal.theta) <
         std::abs(nodes.back().pose.theta - goal.theta));
}

void TestMovesNodeAwayFromObstacle() {
  OptimizerParams params = MakeParams();
  params.goal_position_cost_coefficient = 0.0;
  params.goal_angle_cost_coefficient = 0.0;
  params.max_velocity_change = {10.0, 10.0, 10.0};
  const Optimizer optimizer(params);
  const std::vector<Node> nodes{
      {{-1.0, 0.0, 0.0}, {0.0, 0.0, 0.0}},
      {{0.5, 0.0, 0.0}, {0.0, 0.0, 0.0}},
  };

  const std::vector<Node> optimized =
      optimizer.optimize(nodes, {{0.0, 0.0}}, nodes.back().pose);

  assert(optimized.back().pose.x > nodes.back().pose.x);
}

void TestEscapesObstaclePoint() {
  OptimizerParams params = MakeParams();
  params.goal_position_cost_coefficient = 0.0;
  params.goal_angle_cost_coefficient = 0.0;
  params.max_velocity_change = {10.0, 10.0, 10.0};
  const Optimizer optimizer(params);
  const std::vector<Node> nodes{
      {{-1.0, 0.0, 0.0}, {0.0, 0.0, 0.0}},
      {{0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}},
  };

  const std::vector<Node> optimized =
      optimizer.optimize(nodes, {{0.0, 0.0}}, nodes.back().pose);

  assert(std::hypot(optimized.back().pose.x, optimized.back().pose.y) > 0.0);
}

void TestLimitsVelocityChanges() {
  const OptimizerParams params = MakeParams();
  const Optimizer optimizer(params);
  const std::vector<Node> nodes{
      {{0.0, 0.0, 0.0}, {1.0, -1.0, 0.5}},
      {{5.0, 5.0, 2.0}, {5.0, 5.0, 5.0}},
      {{10.0, -5.0, -2.0}, {-5.0, -5.0, -5.0}},
  };

  const std::vector<Node> optimized =
      optimizer.optimize(nodes, {}, {20.0, 20.0, 3.0});

  assert(optimized.front().pose.x == nodes.front().pose.x);
  assert(optimized.front().twist.x == nodes.front().twist.x);
  for (std::size_t i = 1; i < optimized.size(); ++i) {
    assert(std::abs(optimized[i].twist.x - optimized[i - 1].twist.x) <=
           params.max_velocity_change.x + kTolerance);
    assert(std::abs(optimized[i].twist.y - optimized[i - 1].twist.y) <=
           params.max_velocity_change.y + kTolerance);
    assert(std::abs(optimized[i].twist.theta -
                    optimized[i - 1].twist.theta) <=
           params.max_velocity_change.theta + kTolerance);
  }
}

}  // namespace

int main() {
  TestEmptyPath();
  TestMovesTerminalNodeTowardGoal();
  TestMovesNodeAwayFromObstacle();
  TestEscapesObstaclePoint();
  TestLimitsVelocityChanges();
}
