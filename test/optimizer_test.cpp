#include "maho/optimizer.hpp"

#include <cassert>
#include <cstddef>
#include <cmath>
#include <stdexcept>
#include <utility>

namespace {

constexpr double kTolerance = 1e-9;

OptimizerParams MakeParams() {
  return {
      0.1,
      1e-4,
      {10.0, 10.0, 10.0},
      {10.0, 10.0, 10.0},
  };
}

EvaluationFunction MakeGoalEvaluationFunction() {
  return EvaluationFunction({
      0.0,
      0.0,
      0.0,
      1.0,
      1.0,
      0.0,
      {},
  });
}

EvaluationFunction MakeZeroEvaluationFunction() {
  return EvaluationFunction({
      0.0,
      0.0,
      0.0,
      0.0,
      0.0,
      0.0,
      {},
  });
}

Optimizer MakeOptimizer(
    const OptimizerParams& params,
    const EvaluationFunction& evaluation_function =
        MakeGoalEvaluationFunction()) {
  return Optimizer(params, evaluation_function);
}

bool IsNear(double lhs, double rhs) {
  return std::abs(lhs - rhs) < kTolerance;
}

void AssertInvalidParams(const OptimizerParams& params) {
  bool threw = false;
  try {
    (void)MakeOptimizer(params);
  } catch (const std::invalid_argument&) {
    threw = true;
  }
  assert(threw);
}

void TestEmptyNodes() {
  const Optimizer optimizer = MakeOptimizer(MakeParams());

  assert(optimizer.optimize({}, {}, 1.0, 1.0, {},
                            {1.0, 2.0, 3.0}).empty());
}

void TestOptimizesSingleNodeTowardGoal() {
  const Optimizer optimizer = MakeOptimizer(MakeParams());
  const Nodes nodes{{{0.0, 0.0, 0.0}}};
  const Pose2D goal{1.0, -2.0, 0.5};

  const Nodes optimized = optimizer.optimize(nodes, {}, 1.0, 1.0, {},
                                             goal);

  assert(optimized[0].velocity.x > nodes[0].velocity.x);
  assert(optimized[0].velocity.y < nodes[0].velocity.y);
  assert(optimized[0].velocity.theta > nodes[0].velocity.theta);
}

void TestUsesFirstDt() {
  const Optimizer optimizer = MakeOptimizer(MakeParams());
  const Nodes nodes{{{0.0, 0.0, 0.0}}};

  const Nodes optimized =
      optimizer.optimize(nodes, {}, 1.0, 0.5, {}, {1.0, 0.0, 0.0});

  assert(IsNear(optimized[0].velocity.x, 0.1));
}

void TestOptimizesFirstNodeTowardGoal() {
  const Optimizer optimizer = MakeOptimizer(MakeParams());
  const Nodes nodes{
      {{0.0, 0.0, 0.0}},
      {{0.0, 0.0, 0.0}},
      {{0.0, 0.0, 0.0}},
  };

  const Nodes optimized =
      optimizer.optimize(nodes, {}, 1.0, 1.0, {}, {3.0, 0.0, 0.0});

  assert(optimized.front().velocity.x > nodes.front().velocity.x);
}

void TestKeepsFixedNodes() {
  const Optimizer optimizer = MakeOptimizer(MakeParams());
  const Nodes nodes{
      {{0.0, 0.0, 0.0}},
      {{0.0, 0.0, 0.0}},
      {{0.0, 0.0, 0.0}},
  };

  const Nodes optimized = optimizer.optimize(
      nodes, {}, 1.0, 1.0, {}, {3.0, 0.0, 0.0}, 1);

  assert(IsNear(optimized.front().velocity.x,
                nodes.front().velocity.x));
  assert(IsNear(optimized.front().velocity.y,
                nodes.front().velocity.y));
  assert(IsNear(optimized.front().velocity.theta,
                nodes.front().velocity.theta));
  assert(optimized[1].velocity.x > nodes[1].velocity.x);
}

void TestPerformsOneUpdatePerCall() {
  const Optimizer optimizer = MakeOptimizer(MakeParams());
  const Nodes nodes{{{0.0, 0.0, 0.0}}};
  const Pose2D goal{1.0, 0.0, 0.0};

  const Nodes once = optimizer.optimize(nodes, {}, 1.0, 1.0, {}, goal);
  const Nodes twice = optimizer.optimize(once, {}, 1.0, 1.0, {}, goal);

  assert(IsNear(once[0].velocity.x, 0.2));
  assert(IsNear(twice[0].velocity.x, 0.36));
  assert(twice[0].velocity.x > once[0].velocity.x);
}

void TestMovesAwayFromObstacle() {
  const EvaluationFunction evaluation_function({
      1.0,
      2.0,
      0.0,
      0.0,
      0.0,
      0.0,
      {},
  });
  const Optimizer optimizer =
      MakeOptimizer(MakeParams(), evaluation_function);
  const Nodes nodes{{{0.0, 0.0, 0.0}}};
  const Pose2D initial_pose{0.0, 0.0, 0.0};
  const Env env{{1.0, 0.0}};

  const Nodes optimized =
      optimizer.optimize(nodes, initial_pose, 1.0, 1.0, env,
                         initial_pose);

  assert(std::hypot(optimized[0].velocity.x,
                    optimized[0].velocity.y) > 0.0);
}

void TestLimitsAbsoluteVelocity() {
  OptimizerParams params = MakeParams();
  params.max_velocity = {0.5, 0.6, 0.7};
  const Optimizer optimizer =
      MakeOptimizer(params, MakeZeroEvaluationFunction());
  const Nodes nodes{
      {{5.0, -6.0, 7.0}},
      {{-8.0, 9.0, -10.0}},
  };

  const Nodes optimized = optimizer.optimize(nodes, {}, 1.0, 1.0, {}, {});

  for (const Node& node : optimized) {
    assert(std::abs(node.velocity.x) <=
           params.max_velocity.x + kTolerance);
    assert(std::abs(node.velocity.y) <=
           params.max_velocity.y + kTolerance);
    assert(std::abs(node.velocity.theta) <=
           params.max_velocity.theta + kTolerance);
  }
}

void TestLimitsAdjacentVelocityChanges() {
  OptimizerParams params = MakeParams();
  params.max_velocity_change = {0.1, 0.2, 0.3};
  const Optimizer optimizer =
      MakeOptimizer(params, MakeZeroEvaluationFunction());
  const Nodes nodes{
      {{1.0, -1.0, 0.5}},
      {{5.0, 5.0, 5.0}},
      {{-5.0, -5.0, -5.0}},
  };

  const Nodes optimized = optimizer.optimize(nodes, {}, 1.0, 1.0, {}, {});

  for (std::size_t i = 1; i < optimized.size(); ++i) {
    const Twist2D& previous = optimized[i - 1].velocity;
    const Twist2D& current = optimized[i].velocity;
    assert(std::abs(current.x - previous.x) <=
           params.max_velocity_change.x + kTolerance);
    assert(std::abs(current.y - previous.y) <=
           params.max_velocity_change.y + kTolerance);
    assert(std::abs(current.theta - previous.theta) <=
           params.max_velocity_change.theta + kTolerance);
  }
}

void TestRejectsInvalidParameters() {
  OptimizerParams params = MakeParams();
  params.learning_rate = -0.1;
  AssertInvalidParams(params);

  params = MakeParams();
  params.finite_difference_step = -1e-4;
  AssertInvalidParams(params);

  params = MakeParams();
  params.max_velocity.x = -1.0;
  AssertInvalidParams(params);

  params = MakeParams();
  params.max_velocity.y = -1.0;
  AssertInvalidParams(params);

  params = MakeParams();
  params.max_velocity.theta = -1.0;
  AssertInvalidParams(params);

  params = MakeParams();
  params.max_velocity_change.x = -1.0;
  AssertInvalidParams(params);

  params = MakeParams();
  params.max_velocity_change.y = -1.0;
  AssertInvalidParams(params);

  params = MakeParams();
  params.max_velocity_change.theta = -1.0;
  AssertInvalidParams(params);
}

void TestRejectsInvalidDurations() {
  const Optimizer optimizer = MakeOptimizer(MakeParams());
  for (const auto& durations :
       {std::pair<double, double>{0.0, 0.0},
        std::pair<double, double>{1.0, -0.1},
        std::pair<double, double>{1.0, 1.1}}) {
    bool threw = false;
    try {
      optimizer.optimize({}, {}, durations.first, durations.second, {}, {});
    } catch (const std::invalid_argument&) {
      threw = true;
    }
    assert(threw);
  }
}

void TestRejectsInvalidFixedNodeCount() {
  const Optimizer optimizer = MakeOptimizer(MakeParams());
  bool threw = false;
  try {
    optimizer.optimize({{{0.0, 0.0, 0.0}}}, {}, 1.0, 1.0, {}, {}, 2);
  } catch (const std::invalid_argument&) {
    threw = true;
  }
  assert(threw);
}

}  // namespace

int main() {
  TestEmptyNodes();
  TestOptimizesSingleNodeTowardGoal();
  TestUsesFirstDt();
  TestOptimizesFirstNodeTowardGoal();
  TestKeepsFixedNodes();
  TestPerformsOneUpdatePerCall();
  TestMovesAwayFromObstacle();
  TestLimitsAbsoluteVelocity();
  TestLimitsAdjacentVelocityChanges();
  TestRejectsInvalidParameters();
  TestRejectsInvalidDurations();
  TestRejectsInvalidFixedNodeCount();
}
