#include "maho/evaluation_function.hpp"
#include "maho/kinematics.hpp"

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
                    nodes, {0.0, 0.0, 0.0}, 1.0, 1.0,
                    {{4.0, 0.0}},
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
  const Pose2D first_pose =
      IntegratePose(initial_pose, nodes.front().velocity, 0.25);
  const Pose2D goal =
      IntegratePose(first_pose, nodes.back().velocity, 0.5);

  assert(IsNear(
      evaluation_function.evaluate(nodes, initial_pose, 0.5, 0.25, {},
                                   goal),
      0.0));
}

void TestCalculateTerminalPoseUsesFirstDt() {
  const Nodes nodes{{{1.0, 0.0, 0.0}}, {{2.0, 0.0, 0.0}}};

  assert(IsNear(CalculateTerminalPose({}, nodes, 0.5, 0.25).x, 1.25));
  assert(IsNear(CalculateTerminalPose({}, nodes, 0.5, 0.0).x, 1.0));
}

void TestReturnsFiniteCostForCollision() {
  const EvaluationFunction evaluation_function({2.0, 3.0, 1.0, 0.0,
                                                  0.0, 0.0});
  const Node moving{{1.0, 0.0, 0.0}};

  assert(IsNear(evaluation_function.evaluate(
                    {moving}, {0.0, 0.0, 0.0}, 1.0, 1.0,
                    {{1.5, 0.0}}, {}),
                24.5));
}

void TestUsesMaximumObstacleCostAlongTrajectory() {
  const EvaluationFunction evaluation_function({2.0, 1.0, 0.0, 0.0,
                                                  0.0, 0.0});
  const Node moving{{2.0, 0.0, 0.0}};

  assert(IsNear(evaluation_function.evaluate(
                    {moving}, {}, 1.0, 1.0, {{1.0, 0.0}}, {}),
                2.0));
}

void TestUsesMaximumObstacleCostAlongArc() {
  constexpr double kHalfPi = 1.57079632679489661923;
  const EvaluationFunction evaluation_function({2.0, 0.1, 0.0, 0.0,
                                                  0.0, 0.0});
  const Node moving{{1.0, 0.0, kHalfPi}};
  const Point2D midpoint{
      std::sin(kHalfPi / 2.0) / kHalfPi,
      (1.0 - std::cos(kHalfPi / 2.0)) / kHalfPi,
  };

  assert(IsNear(evaluation_function.evaluate(
                    {moving}, {}, 1.0, 1.0, {midpoint}, {}),
                0.02));
}

void TestAddsGoalVelocityCostForAllNodes() {
  const EvaluationFunction evaluation_function =
      MakeEvaluationFunction({2.0, 0.5, 1.5, 3.0, 0.1});
  const Nodes nodes{{{0.0, 0.0, 0.0}}, {{1.0, 0.0, 0.0}}};

  assert(IsNear(evaluation_function.evaluate(
                    nodes, {0.0, 0.0, 0.0}, 1.0, 1.0, {},
                    {10.0, 0.0, 0.0}),
                2.5));
}

void TestPenalizesTranslationalSpeedWithinGoalTolerance() {
  const EvaluationFunction evaluation_function =
      MakeEvaluationFunction({2.0, 0.5, 1.5, 3.0, 0.1});
  const Node node{{3.0, 4.0, 0.0}};

  assert(IsNear(evaluation_function.evaluate(
                    {node}, {6.95, -4.0, 0.0}, 1.0, 1.0, {},
                    {10.0, 0.0, 0.0}),
                50.0));
}

void TestUsesOriginalVelocityWhenFirstDtIsZero() {
  const EvaluationFunction evaluation_function =
      MakeEvaluationFunction({2.0, 0.0, 0.0, 0.0, 10.0});
  const Node node{{3.0, 0.0, 0.0}};

  assert(IsNear(evaluation_function.evaluate(
                    {node}, {}, 1.0, 0.0, {}, {}),
                18.0));
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
    evaluation_function.evaluate({{{0.0, 0.0, 0.0}}}, {}, 0.0, 0.0,
                                 {}, {});
  } catch (const std::invalid_argument&) {
    threw = true;
  }
  assert(threw);
}

void TestRejectsInvalidFirstDt() {
  const EvaluationFunction evaluation_function = MakeEvaluationFunction();
  for (const double first_dt : {-0.1, 1.1}) {
    bool threw = false;
    try {
      evaluation_function.evaluate({}, {}, 1.0, first_dt, {}, {});
    } catch (const std::invalid_argument&) {
      threw = true;
    }
    assert(threw);
  }
}

void TestCalculateTerminalPoseRejectsInvalidDt() {
  for (const double first_dt : {-0.1, 1.1}) {
    bool threw = false;
    try {
      CalculateTerminalPose({}, {}, 1.0, first_dt);
    } catch (const std::invalid_argument&) {
      threw = true;
    }
    assert(threw);
  }
}

void TestAddsTerminalVelocityCost() {
  const EvaluationFunction evaluation_function =
      MakeEvaluationFunction({0.0, 0.0, 0.0, 0.0, 0.0, 4.0, 5.0});
  const Node node{{3.0, 4.0, 2.0}};
  const Pose2D initial_pose{-3.0, -4.0, 0.0};
  const Pose2D goal = IntegratePose(initial_pose, node.velocity, 1.0);

  assert(IsNear(evaluation_function.evaluate(
                    {node}, initial_pose, 1.0, 1.0, {}, goal),
                120.0));
}

void TestAddsDistanceBasedAngularVelocityCost() {
  const EvaluationFunction evaluation_function = MakeEvaluationFunction(
      {1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 2.0, 1.0, 1.5});
  const Node node{{0.0, 0.0, 0.5}};

  assert(IsNear(evaluation_function.evaluate(
                    {node}, {0.0, 0.0, 0.0}, 1.0, 1.0, {},
                    {10.0, 0.0, 0.0}),
                2.0));
}

void TestDoesNotAddTerminalVelocityCostFarFromGoal() {
  const EvaluationFunction evaluation_function =
      MakeEvaluationFunction({0.0, 0.5, 1.0, 0.0, 0.1, 4.0, 5.0});
  const Node node{{3.0, 4.0, 2.0}};

  assert(IsNear(evaluation_function.evaluate(
                    {node}, {0.0, 0.0, 0.0}, 1.0, 1.0, {},
                    {10.0, 0.0, 0.0}),
                0.0));
}

}  // namespace

int main() {
  TestCalculatesSharedCosts();
  TestUsesIntegratedPose();
  TestCalculateTerminalPoseUsesFirstDt();
  TestReturnsFiniteCostForCollision();
  TestUsesMaximumObstacleCostAlongTrajectory();
  TestUsesMaximumObstacleCostAlongArc();
  TestAddsGoalVelocityCostForAllNodes();
  TestPenalizesTranslationalSpeedWithinGoalTolerance();
  TestUsesOriginalVelocityWhenFirstDtIsZero();
  TestRejectsNegativeParameter();
  TestRejectsInvalidDt();
  TestRejectsInvalidFirstDt();
  TestCalculateTerminalPoseRejectsInvalidDt();
  TestAddsTerminalVelocityCost();
  TestAddsDistanceBasedAngularVelocityCost();
  TestDoesNotAddTerminalVelocityCostFarFromGoal();
}
