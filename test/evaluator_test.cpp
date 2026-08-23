#include "maho/evaluator.hpp"

#include <cassert>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace {

bool IsNear(double actual, double expected) {
  return std::abs(actual - expected) < 1e-12;
}

void TestAddsGoalVelocityCostForAllNodes() {
  const Evaluator evaluator(
      {0.0, 0.0, 0.0, {2.0, 0.5, 1.5, 3.0, 0.1}});
  constexpr double kHalfPi = 1.57079632679489661923;
  const Nodes nodes{
      {{0.0, 0.0, 0.0}, {2.0, 1.0, 0.0}},
      {{9.0, 0.0, kHalfPi}, {1.0, -2.0, 0.0}},
  };

  assert(IsNear(evaluator.evaluate(nodes, {}, {10.0, 0.0, 0.0}), 8.5));
}

void TestPenalizesTranslationalSpeedWithinGoalTolerance() {
  const Evaluator evaluator(
      {0.0, 0.0, 0.0, {2.0, 0.5, 1.5, 3.0, 0.1}});
  const Node node{{9.95, 0.0, 0.0}, {3.0, 4.0, 0.0}};

  assert(IsNear(evaluator.evaluate({node}, {}, {10.0, 0.0, 0.0}),
                    50.0));
}

void TestRejectsNegativeGoalVelocityCostParameter() {
  bool threw = false;
  try {
    const Evaluator evaluator(
        {0.0, 0.0, 0.0, {1.0, -0.5, 1.5, 1.0, 0.1}});
  } catch (const std::invalid_argument&) {
    threw = true;
  }
  assert(threw);
}

void TestAddsTerminalVelocityCost() {
  const Evaluator evaluator(
      {0.0, 0.0, 0.0, {0.0, 0.0, 0.0, 0.0, 0.0, 4.0, 5.0}});
  const Node node{{0.0, 0.0, 0.0}, {3.0, 4.0, 2.0}};

  assert(IsNear(evaluator.evaluate({node}, {}, {0.0, 0.0, 0.0}),
                    120.0));
}

void TestAddsDistanceBasedAngularVelocityCost() {
  const Evaluator evaluator(
      {0.0, 0.0, 0.0,
       {1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 2.0, 1.0, 1.5}});
  const Node node{{0.0, 0.0, 0.5}, {0.0, 0.0, 0.5}};

  assert(IsNear(evaluator.evaluate({node}, {}, {10.0, 0.0, 0.0}), 2.0));
}

void TestDoesNotAddTerminalVelocityCostFarFromGoal() {
  const Evaluator evaluator(
      {0.0, 0.0, 0.0, {0.0, 0.5, 1.0, 0.0, 0.1, 4.0, 5.0}});
  const Node node{{0.0, 0.0, 0.0}, {3.0, 4.0, 2.0}};

  assert(IsNear(evaluator.evaluate({node}, {}, {10.0, 0.0, 0.0}), 0.0));
}

}  // namespace

int main() {
  const Evaluator evaluator({2.0, 1.0, 3.0});
  const Node origin{{0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}};

  assert(std::isinf(evaluator.evaluate({}, {}, {0.0, 0.0, 0.0})));
  assert(IsNear(evaluator.evaluate({origin}, {{3.0, 0.0}},
                                   {3.0, 4.0, 0.0}),
                    16.0));

  const Node second{{1.0, 0.0, 0.0}, {0.0, 0.0, 0.0}};
  assert(IsNear(evaluator.evaluate({origin, second}, {{4.0, 0.0}},
                                   {1.0, 0.0, 0.0}),
                    5.0 / 3.0));

  const Node rotated{{0.0, 0.0, 3.0}, {0.0, 0.0, 0.0}};
  constexpr double kExpectedAngleDifference = 0.28318530717958647692;
  assert(IsNear(evaluator.evaluate({rotated}, {}, {0.0, 0.0, -3.0}),
                    3.0 * kExpectedAngleDifference));

  assert(std::isinf(
      evaluator.evaluate({origin}, {{1.0, 0.0}}, {0.0, 0.0, 0.0})));
  assert(std::isinf(
      evaluator.evaluate({origin}, {{0.5, 0.0}}, {0.0, 0.0, 0.0})));

  TestAddsGoalVelocityCostForAllNodes();
  TestPenalizesTranslationalSpeedWithinGoalTolerance();
  TestRejectsNegativeGoalVelocityCostParameter();
  TestAddsTerminalVelocityCost();
  TestAddsDistanceBasedAngularVelocityCost();
  TestDoesNotAddTerminalVelocityCostFarFromGoal();
}
