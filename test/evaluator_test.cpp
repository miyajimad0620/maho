#include "maho/evaluator.hpp"

#include <cassert>
#include <cmath>
#include <stdexcept>

namespace {

bool IsNear(double actual, double expected) {
  return std::abs(actual - expected) < 1e-12;
}

EvaluationFunction MakeEvaluationFunction(
    const GoalVelocityCostParams& goal_velocity_cost = {}) {
  return EvaluationFunction({
      0.0,
      0.0,
      0.0,
      0.0,
      0.0,
      0.0,
      goal_velocity_cost,
  });
}

void TestCalculatesSharedCosts() {
  const EvaluationFunction evaluation_function({
      2.0,
      3.0,
      1.0,
      3.0,
      5.0,
      7.0,
  });
  const Nodes nodes{{{0.0, 0.0, 0.0}}, {{1.0, 0.0, 0.0}}};

  assert(IsNear(evaluation_function.evaluate(
                    nodes, {0.0, 0.0, 0.0}, 1.0, {{4.0, 0.0}},
                    {2.0, 0.0, 0.0}),
                12.0));
}

void TestUsesIntegratedPose() {
  const EvaluationFunction evaluation_function({
      0.0,
      0.0,
      0.0,
      1.0,
      1.0,
      0.0,
  });
  constexpr double kHalfPi = 1.57079632679489661923;
  const Nodes nodes{{{2.0, 0.0, 1.0}}, {{0.0, 2.0, 0.0}}};
  const Pose2D initial_pose{1.0, 2.0, kHalfPi};
  const Pose2D first_pose{
      initial_pose.x + std::cos(initial_pose.theta),
      initial_pose.y + std::sin(initial_pose.theta),
      initial_pose.theta + 0.5,
  };
  const Pose2D goal{
      first_pose.x - std::sin(first_pose.theta),
      first_pose.y + std::cos(first_pose.theta),
      first_pose.theta,
  };

  assert(IsNear(
      evaluation_function.evaluate(nodes, initial_pose, 0.5, {}, goal),
      0.0));
}

void TestSwitchesCollisionHandlingAtIntegratedPose() {
  const EvaluationFunction evaluation_function({2.0, 3.0, 1.0, 0.0,
                                                  0.0, 0.0});
  const Node moving{{1.0, 0.0, 0.0}};
  const Pose2D initial_pose{0.0, 0.0, 0.0};
  const Pose2D goal{0.0, 0.0, 0.0};

  assert(std::isinf(evaluation_function.evaluate(
      {moving}, initial_pose, 1.0, {{1.5, 0.0}}, goal,
      CollisionHandling::kReturnInfinity)));
  assert(IsNear(evaluation_function.evaluate(
                    {moving}, initial_pose, 1.0, {{1.5, 0.0}}, goal,
                    CollisionHandling::kUseFiniteCost),
                24.5));
}

void TestEvaluatorReturnsInfinityForCollision() {
  const Evaluator evaluator(
      EvaluationFunction({2.0, 3.0, 1.0, 0.0, 0.0, 0.0}));
  const Pose2D initial_pose{0.0, 0.0, 0.0};
  const Pose2D goal{0.0, 0.0, 0.0};

  assert(std::isinf(evaluator.evaluate({}, initial_pose, 1.0, {}, goal)));
  assert(std::isinf(evaluator.evaluate(
      {{{1.0, 0.0, 0.0}}}, initial_pose, 1.0, {{1.5, 0.0}}, goal)));
}

void TestAddsGoalVelocityCostForAllNodes() {
  const Evaluator evaluator(
      MakeEvaluationFunction({2.0, 0.5, 1.5, 3.0, 0.1}));
  const Nodes nodes{{{0.0, 0.0, 0.0}}, {{1.0, 0.0, 0.0}}};

  assert(IsNear(evaluator.evaluate(nodes, {0.0, 0.0, 0.0}, 1.0, {},
                                    {10.0, 0.0, 0.0}),
                2.5));
}

void TestPenalizesTranslationalSpeedWithinGoalTolerance() {
  const Evaluator evaluator(
      MakeEvaluationFunction({2.0, 0.5, 1.5, 3.0, 0.1}));
  const Node node{{3.0, 4.0, 0.0}};

  assert(IsNear(evaluator.evaluate({node}, {6.95, -4.0, 0.0}, 1.0, {},
                                    {10.0, 0.0, 0.0}),
                50.0));
}

void TestRejectsNegativeParameter() {
  bool threw = false;
  try {
    const EvaluationFunction evaluation_function(
        {0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
         {1.0, -0.5, 1.5, 1.0, 0.1}});
  } catch (const std::invalid_argument&) {
    threw = true;
  }
  assert(threw);
}

void TestRejectsInvalidDt() {
  const EvaluationFunction evaluation_function = MakeEvaluationFunction();
  bool threw = false;
  try {
    evaluation_function.evaluate({{{0.0, 0.0, 0.0}}}, {}, 0.0, {}, {});
  } catch (const std::invalid_argument&) {
    threw = true;
  }
  assert(threw);
}

void TestAddsTerminalVelocityCost() {
  const Evaluator evaluator(
      MakeEvaluationFunction({0.0, 0.0, 0.0, 0.0, 0.0, 4.0, 5.0}));
  const Node node{{3.0, 4.0, 2.0}};

  assert(IsNear(evaluator.evaluate({node}, {-3.0, -4.0, 0.0}, 1.0, {},
                                    {0.0, 0.0, 0.0}),
                120.0));
}

void TestAddsDistanceBasedAngularVelocityCost() {
  const Evaluator evaluator(MakeEvaluationFunction(
      {1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 2.0, 1.0, 1.5}));
  const Node node{{0.0, 0.0, 0.5}};

  assert(IsNear(evaluator.evaluate({node}, {0.0, 0.0, 0.0}, 1.0, {},
                                    {10.0, 0.0, 0.0}),
                2.0));
}

void TestDoesNotAddTerminalVelocityCostFarFromGoal() {
  const Evaluator evaluator(
      MakeEvaluationFunction({0.0, 0.5, 1.0, 0.0, 0.1, 4.0, 5.0}));
  const Node node{{3.0, 4.0, 2.0}};

  assert(IsNear(evaluator.evaluate({node}, {0.0, 0.0, 0.0}, 1.0, {},
                                    {10.0, 0.0, 0.0}),
                0.0));
}

}  // namespace

int main() {
  TestCalculatesSharedCosts();
  TestUsesIntegratedPose();
  TestSwitchesCollisionHandlingAtIntegratedPose();
  TestEvaluatorReturnsInfinityForCollision();
  TestAddsGoalVelocityCostForAllNodes();
  TestPenalizesTranslationalSpeedWithinGoalTolerance();
  TestRejectsNegativeParameter();
  TestRejectsInvalidDt();
  TestAddsTerminalVelocityCost();
  TestAddsDistanceBasedAngularVelocityCost();
  TestDoesNotAddTerminalVelocityCostFarFromGoal();
}
