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
  const Nodes nodes{
      {{0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}},
      {{1.0, 0.0, 0.0}, {1.0, 0.0, 0.0}},
  };

  assert(IsNear(evaluation_function.evaluate(
                    nodes, {{4.0, 0.0}}, {2.0, 0.0, 0.0}),
                12.0));
}

void TestSwitchesCollisionHandling() {
  const EvaluationFunction evaluation_function({2.0, 3.0, 1.0, 0.0,
                                                  0.0, 0.0});
  const Node origin{{0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}};

  assert(std::isinf(evaluation_function.evaluate(
      {origin}, {{0.5, 0.0}}, origin.pose,
      CollisionHandling::kReturnInfinity)));
  assert(IsNear(evaluation_function.evaluate(
                    {origin}, {{0.5, 0.0}}, origin.pose,
                    CollisionHandling::kUseFiniteCost),
                24.5));
}

void TestEvaluatorReturnsInfinityForCollision() {
  const EvaluationFunction evaluation_function({2.0, 3.0, 1.0, 0.0,
                                                  0.0, 0.0});
  const Evaluator evaluator(evaluation_function);
  const Node origin{{0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}};

  assert(std::isinf(evaluator.evaluate({}, {}, origin.pose)));
  assert(std::isinf(
      evaluator.evaluate({origin}, {{0.5, 0.0}}, origin.pose)));
}

void TestAddsGoalVelocityCostForAllNodes() {
  const Evaluator evaluator(
      MakeEvaluationFunction({2.0, 0.5, 1.5, 3.0, 0.1}));
  constexpr double kHalfPi = 1.57079632679489661923;
  const Nodes nodes{
      {{0.0, 0.0, 0.0}, {2.0, 1.0, 0.0}},
      {{9.0, 0.0, kHalfPi}, {1.0, -2.0, 0.0}},
  };

  assert(IsNear(evaluator.evaluate(nodes, {}, {10.0, 0.0, 0.0}), 8.5));
}

void TestPenalizesTranslationalSpeedWithinGoalTolerance() {
  const Evaluator evaluator(
      MakeEvaluationFunction({2.0, 0.5, 1.5, 3.0, 0.1}));
  const Node node{{9.95, 0.0, 0.0}, {3.0, 4.0, 0.0}};

  assert(IsNear(evaluator.evaluate({node}, {}, {10.0, 0.0, 0.0}),
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

void TestAddsTerminalVelocityCost() {
  const Evaluator evaluator(
      MakeEvaluationFunction({0.0, 0.0, 0.0, 0.0, 0.0, 4.0, 5.0}));
  const Node node{{0.0, 0.0, 0.0}, {3.0, 4.0, 2.0}};

  assert(IsNear(evaluator.evaluate({node}, {}, {0.0, 0.0, 0.0}),
                120.0));
}

void TestAddsDistanceBasedAngularVelocityCost() {
  const Evaluator evaluator(MakeEvaluationFunction(
      {1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 2.0, 1.0, 1.5}));
  const Node node{{0.0, 0.0, 0.5}, {0.0, 0.0, 0.5}};

  assert(IsNear(evaluator.evaluate({node}, {}, {10.0, 0.0, 0.0}), 2.0));
}

void TestDoesNotAddTerminalVelocityCostFarFromGoal() {
  const Evaluator evaluator(
      MakeEvaluationFunction({0.0, 0.5, 1.0, 0.0, 0.1, 4.0, 5.0}));
  const Node node{{0.0, 0.0, 0.0}, {3.0, 4.0, 2.0}};

  assert(IsNear(evaluator.evaluate({node}, {}, {10.0, 0.0, 0.0}), 0.0));
}

}  // namespace

int main() {
  TestCalculatesSharedCosts();
  TestSwitchesCollisionHandling();
  TestEvaluatorReturnsInfinityForCollision();
  TestAddsGoalVelocityCostForAllNodes();
  TestPenalizesTranslationalSpeedWithinGoalTolerance();
  TestRejectsNegativeParameter();
  TestAddsTerminalVelocityCost();
  TestAddsDistanceBasedAngularVelocityCost();
  TestDoesNotAddTerminalVelocityCostFarFromGoal();
}
