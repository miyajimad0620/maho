#include "maho/optimizer.hpp"

#include "maho/expander.hpp"

#include <cassert>
#include <cmath>
#include <stdexcept>
#include <vector>

namespace {

constexpr double kTolerance = 1e-9;

OptimizerParams MakeParams() {
  return {
      0.1,             // learning_rate
      1e-4,            // finite_difference_step
      20,              // max_iterations
      1.0,             // dt
      {2.0, 2.0, 2.0},
      {0.1, 0.2, 0.3}  // max_velocity_change
  };
}

EvaluationFunctionParams MakeEvaluationParams() {
  return {
      1.0,
      1.0,
      0.2,
      1.0,
      1.0,
      0.1,
  };
}

Optimizer MakeOptimizer(
    const OptimizerParams& params,
    const EvaluationFunctionParams& evaluation_params =
        MakeEvaluationParams()) {
  return Optimizer(params, EvaluationFunction(evaluation_params));
}

void TestEmptyPath() {
  const Optimizer optimizer = MakeOptimizer(MakeParams());
  assert(optimizer.optimize({}, {}, {1.0, 2.0, 3.0}).empty());
}

void TestMovesTerminalNodeTowardGoal() {
  OptimizerParams params = MakeParams();
  params.max_velocity_change = {10.0, 10.0, 10.0};
  params.max_velocity = {10.0, 10.0, 10.0};
  const Optimizer optimizer = MakeOptimizer(params);
  const std::vector<Node> nodes{
      {{0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}},
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
  EvaluationFunctionParams evaluation_params = MakeEvaluationParams();
  evaluation_params.goal_position_cost_coefficient = 0.0;
  evaluation_params.goal_angle_cost_coefficient = 0.0;
  params.max_velocity_change = {10.0, 10.0, 10.0};
  params.max_velocity = {10.0, 10.0, 10.0};
  const Optimizer optimizer = MakeOptimizer(params, evaluation_params);
  const std::vector<Node> nodes{
      {{-2.0, 0.0, 0.0}, {1.0, 0.0, 0.0}},
      {{-1.0, 0.0, 0.0}, {1.0, 0.0, 0.0}},
      {{0.5, 0.0, 0.0}, {1.0, 0.0, 0.0}},
  };

  const std::vector<Node> optimized =
      optimizer.optimize(nodes, {{0.0, 0.0}}, nodes.back().pose);

  assert(optimized.back().pose.x > nodes.back().pose.x);
}

void TestEscapesObstaclePoint() {
  OptimizerParams params = MakeParams();
  EvaluationFunctionParams evaluation_params = MakeEvaluationParams();
  evaluation_params.goal_position_cost_coefficient = 0.0;
  evaluation_params.goal_angle_cost_coefficient = 0.0;
  params.max_velocity_change = {10.0, 10.0, 10.0};
  params.max_velocity = {10.0, 10.0, 10.0};
  const Optimizer optimizer = MakeOptimizer(params, evaluation_params);
  const std::vector<Node> nodes{
      {{-2.0, 0.0, 0.0}, {1.0, 0.0, 0.0}},
      {{-1.0, 0.0, 0.0}, {1.0, 0.0, 0.0}},
      {{0.0, 0.0, 0.0}, {1.0, 0.0, 0.0}},
  };

  const std::vector<Node> optimized =
      optimizer.optimize(nodes, {{0.0, 0.0}}, nodes.back().pose);

  assert(std::hypot(optimized.back().pose.x, optimized.back().pose.y) > 0.0);
}

void TestLimitsVelocityChanges() {
  const OptimizerParams params = MakeParams();
  const Optimizer optimizer = MakeOptimizer(params);
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

    const Node& previous = optimized[i - 1];
    const Node& current = optimized[i];
    const double cos_theta = std::cos(previous.pose.theta);
    const double sin_theta = std::sin(previous.pose.theta);
    assert(std::abs(current.pose.x -
                    (previous.pose.x +
                     (cos_theta * current.twist.x -
                      sin_theta * current.twist.y) *
                         params.dt)) < kTolerance);
    assert(std::abs(current.pose.y -
                    (previous.pose.y +
                     (sin_theta * current.twist.x +
                      cos_theta * current.twist.y) *
                         params.dt)) < kTolerance);
    assert(std::abs(current.pose.theta -
                    (previous.pose.theta +
                     current.twist.theta * params.dt)) < kTolerance);
  }
}

void TestUsesCurrentNodeVelocity() {
  OptimizerParams params = MakeParams();
  params.max_iterations = 0;
  params.dt = 0.5;
  params.max_velocity_change = {10.0, 10.0, 10.0};
  params.max_velocity = {10.0, 10.0, 10.0};
  const Optimizer optimizer = MakeOptimizer(params);
  constexpr double kHalfPi = 1.57079632679489661923;
  const std::vector<Node> nodes{
      {{1.0, 2.0, kHalfPi}, {0.0, 0.0, 0.0}},
      {{0.5, 3.0, kHalfPi + 0.25}, {-3.0, -3.0, -3.0}},
  };

  const std::vector<Node> optimized =
      optimizer.optimize(nodes, {}, {0.0, 0.0, 0.0});

  assert(std::abs(optimized[1].pose.x - 0.5) < kTolerance);
  assert(std::abs(optimized[1].pose.y - 3.0) < kTolerance);
  assert(std::abs(optimized[1].pose.theta -
                  (kHalfPi + 0.25)) < kTolerance);
  assert(std::abs(optimized[1].twist.x - 2.0) < kTolerance);
  assert(std::abs(optimized[1].twist.y - 1.0) < kTolerance);
  assert(std::abs(optimized[1].twist.theta - 0.5) < kTolerance);
}

void TestPreservesExpandedNodeKinematics() {
  OptimizerParams params = MakeParams();
  params.max_iterations = 0;
  params.dt = 0.2;
  params.max_velocity = {10.0, 10.0, 10.0};
  params.max_velocity_change = {0.2, 0.2, 0.15};
  const Optimizer optimizer = MakeOptimizer(params);
  const Expander expander({0.2, {0.2, 0.2, 0.15}});
  const Node initial{{0.3, -0.2, 0.4}, {0.5, -0.1, 0.2}};

  for (const Node& expanded : expander.expand(initial)) {
    const std::vector<Node> optimized =
        optimizer.optimize({initial, expanded}, {}, expanded.pose);
    assert(std::abs(optimized[1].pose.x - expanded.pose.x) < kTolerance);
    assert(std::abs(optimized[1].pose.y - expanded.pose.y) < kTolerance);
    assert(std::abs(optimized[1].pose.theta - expanded.pose.theta) <
           kTolerance);
    assert(std::abs(optimized[1].twist.x - expanded.twist.x) < kTolerance);
    assert(std::abs(optimized[1].twist.y - expanded.twist.y) < kTolerance);
    assert(std::abs(optimized[1].twist.theta - expanded.twist.theta) <
           kTolerance);
  }
}

void TestExpandsSimultaneousBrakingNode() {
  const Expander expander({0.2, {0.2, 0.2, 0.15}});
  const Node initial{{0.3, -0.2, 0.4}, {0.5, -0.3, 0.2}};

  const Node& braking = expander.expand(initial).back();

  assert(std::abs(braking.twist.x - 0.3) < kTolerance);
  assert(std::abs(braking.twist.y + 0.1) < kTolerance);
  assert(std::abs(braking.twist.theta - 0.05) < kTolerance);
}

void TestLimitsAbsoluteVelocity() {
  OptimizerParams params = MakeParams();
  params.max_velocity = {0.5, 0.6, 0.7};
  params.max_velocity_change = {10.0, 10.0, 10.0};
  const Optimizer optimizer = MakeOptimizer(params);
  const std::vector<Node> nodes{
      {{0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}},
      {{0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}},
      {{100.0, -100.0, 100.0}, {100.0, -100.0, 100.0}},
  };

  const std::vector<Node> optimized =
      optimizer.optimize(nodes, {}, {100.0, -100.0, 100.0});

  for (std::size_t i = 1; i < optimized.size(); ++i) {
    assert(std::abs(optimized[i].twist.x) <=
           params.max_velocity.x + kTolerance);
    assert(std::abs(optimized[i].twist.y) <=
           params.max_velocity.y + kTolerance);
    assert(std::abs(optimized[i].twist.theta) <=
           params.max_velocity.theta + kTolerance);
  }
}

void TestRejectsNegativeMaxVelocity() {
  OptimizerParams params = MakeParams();
  params.max_velocity.x = -1.0;

  bool threw = false;
  try {
    const Optimizer optimizer = MakeOptimizer(params);
  } catch (const std::invalid_argument&) {
    threw = true;
  }
  assert(threw);
}

void TestMovesVelocityTowardDistanceBasedTarget() {
  OptimizerParams params = MakeParams();
  EvaluationFunctionParams evaluation_params = MakeEvaluationParams();
  evaluation_params.obstacle_cost_coefficient = 0.0;
  evaluation_params.goal_position_cost_coefficient = 0.0;
  evaluation_params.goal_angle_cost_coefficient = 0.0;
  evaluation_params.velocity_change_cost_coefficient = 0.0;
  params.learning_rate = 0.01;
  params.max_iterations = 100;
  params.max_velocity = {10.0, 10.0, 10.0};
  params.max_velocity_change = {10.0, 10.0, 10.0};
  evaluation_params.goal_velocity_cost = {1.0, 0.1, 1.0, 1.0, 0.1};
  const std::vector<Node> nodes{
      {{0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}},
      {{0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}},
      {{0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}},
  };

  const std::vector<Node> optimized =
      MakeOptimizer(params, evaluation_params)
          .optimize(nodes, {}, {10.0, 0.0, 0.0});

  assert(optimized[1].twist.x > nodes[1].twist.x);
}

double SquaredVelocityChanges(const std::vector<Node>& nodes) {
  double total = 0.0;
  for (std::size_t i = 1; i < nodes.size(); ++i) {
    const double dx = nodes[i].twist.x - nodes[i - 1].twist.x;
    const double dy = nodes[i].twist.y - nodes[i - 1].twist.y;
    const double dtheta =
        nodes[i].twist.theta - nodes[i - 1].twist.theta;
    total += dx * dx + dy * dy + dtheta * dtheta;
  }
  return total;
}

void TestPenalizesVelocityChanges() {
  OptimizerParams params = MakeParams();
  EvaluationFunctionParams evaluation_params = MakeEvaluationParams();
  evaluation_params.obstacle_cost_coefficient = 0.0;
  evaluation_params.goal_angle_cost_coefficient = 0.0;
  params.learning_rate = 0.01;
  params.max_iterations = 100;
  params.max_velocity = {10.0, 10.0, 10.0};
  params.max_velocity_change = {10.0, 10.0, 10.0};
  const std::vector<Node> nodes{
      {{0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}},
      {{0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}},
      {{0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}},
      {{0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}},
  };
  const Pose2D goal{3.0, 0.0, 0.0};

  evaluation_params.velocity_change_cost_coefficient = 0.0;
  const std::vector<Node> unpenalized =
      MakeOptimizer(params, evaluation_params).optimize(nodes, {}, goal);
  evaluation_params.velocity_change_cost_coefficient = 1.0;
  const std::vector<Node> penalized =
      MakeOptimizer(params, evaluation_params).optimize(nodes, {}, goal);

  assert(SquaredVelocityChanges(penalized) <
         SquaredVelocityChanges(unpenalized));
  assert(penalized.back().pose.x > nodes.back().pose.x);
}

}  // namespace

int main() {
  TestEmptyPath();
  TestMovesTerminalNodeTowardGoal();
  TestMovesNodeAwayFromObstacle();
  TestEscapesObstaclePoint();
  TestLimitsVelocityChanges();
  TestUsesCurrentNodeVelocity();
  TestPreservesExpandedNodeKinematics();
  TestExpandsSimultaneousBrakingNode();
  TestLimitsAbsoluteVelocity();
  TestRejectsNegativeMaxVelocity();
  TestMovesVelocityTowardDistanceBasedTarget();
  TestPenalizesVelocityChanges();
}
