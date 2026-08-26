#include "maho/maho.hpp"

#include <cassert>
#include <cmath>
#include <cstddef>
#include <stdexcept>

namespace {

constexpr double kTolerance = 1e-9;

EvaluationFunction MakeEvaluationFunction() {
  return EvaluationFunction({0.0, 0.0, 0.1, 1.0, 0.1, 0.05});
}

Maho MakeMaho(std::size_t replan_optimization_count = 0,
              std::size_t pose_update_optimization_count = 0,
              const Env& env = {}, double robot_radius = 0.1) {
  const MahoParams params{
      {0.0, 0.0, 0.0},
      {{0.0, 0.0, 0.0}},
      env,
      {3.0, 1.0, 0.2},
      0.2,
      replan_optimization_count,
      pose_update_optimization_count,
      {0.2, 0.1, 0.1, 0.1},
  };
  const Expander expander(ExpanderParams{{0.2, 0.2, 0.1}});
  const EvaluationFunction evaluation_function = MakeEvaluationFunction();
  const CollisionDetector collision_detector({robot_radius});
  const Selector selector({1.0, 0.5, 0.2}, evaluation_function);
  const Optimizer optimizer({
      0.02,
      1e-4,
      {2.0, 2.0, 1.0},
      {0.2, 0.2, 0.1},
  }, evaluation_function);
  return Maho(params, expander, collision_detector, selector, optimizer);
}

bool IsSameNode(const Node& lhs, const Node& rhs) {
  return std::abs(lhs.velocity.x - rhs.velocity.x) < kTolerance &&
         std::abs(lhs.velocity.y - rhs.velocity.y) < kTolerance &&
         std::abs(lhs.velocity.theta - rhs.velocity.theta) < kTolerance;
}

bool IsSameNodes(const Nodes& lhs, const Nodes& rhs) {
  if (lhs.size() != rhs.size()) {
    return false;
  }
  for (std::size_t i = 0; i < lhs.size(); ++i) {
    if (!IsSameNode(lhs[i], rhs[i])) {
      return false;
    }
  }
  return true;
}

bool Contains(const Maho::NodeSequences& node_sequences,
              const Nodes& expected) {
  for (const Nodes& nodes : node_sequences) {
    if (IsSameNodes(nodes, expected)) {
      return true;
    }
  }
  return false;
}

void AssertFixedSize(const Maho::NodeSequences& node_sequences) {
  assert(node_sequences.size() == Maho::kNodeSequenceCount);
  for (const Nodes& nodes : node_sequences) {
    assert(nodes.size() == Maho::kNodeSequenceLength);
  }
}

void AssertSortedForPose(const Maho::NodeSequences& node_sequences,
                         const Pose2D& pose) {
  const EvaluationFunction evaluation_function = MakeEvaluationFunction();
  double previous_cost = -1.0;
  for (const Nodes& nodes : node_sequences) {
    const double cost = evaluation_function.evaluate(
        nodes, pose, 0.2, {}, {3.0, 1.0, 0.2});
    assert(previous_cost <= cost + kTolerance);
    previous_cost = cost;
  }
}

void TestConstructsFixedLengthNodeSequences() {
  const Maho maho = MakeMaho();
  const Maho::NodeSequences node_sequences = maho.get_nodes();

  AssertFixedSize(node_sequences);
  AssertSortedForPose(node_sequences, {0.0, 0.0, 0.0});
}

void TestReplanAppendsThenDropsExpansionWhenOptimizationIsDisabled() {
  Maho maho = MakeMaho();
  const Maho::NodeSequences before = maho.get_nodes();
  const Pose2D pose{0.5, -0.2, 0.1};

  maho.replan(pose);

  const Maho::NodeSequences after = maho.get_nodes();
  AssertFixedSize(after);
  for (const Nodes& nodes : after) {
    assert(Contains(before, nodes));
  }
  AssertSortedForPose(after, pose);
}

void TestUpdatePosePreservesNodesWhenOptimizationIsDisabled() {
  Maho maho = MakeMaho();
  const Maho::NodeSequences before = maho.get_nodes();
  const Pose2D pose{1.0, 0.5, -0.2};

  maho.update_pose(pose);

  const Maho::NodeSequences after = maho.get_nodes();
  for (const Nodes& nodes : after) {
    assert(Contains(before, nodes));
  }
  AssertSortedForPose(after, pose);
}

void TestUpdatePoseRunsConfiguredOptimizationCount() {
  Maho maho = MakeMaho(0, 1);
  const Maho::NodeSequences before = maho.get_nodes();

  maho.update_pose({0.0, 0.0, 0.0});

  const Maho::NodeSequences after = maho.get_nodes();
  bool changed = false;
  for (const Nodes& nodes : after) {
    changed = changed || !Contains(before, nodes);
  }
  assert(changed);
}

void TestReplanRunsConfiguredOptimizationCount() {
  Maho maho = MakeMaho(1, 0);
  const Maho::NodeSequences before = maho.get_nodes();

  maho.replan({0.3, 0.0, 0.0});

  const Maho::NodeSequences after = maho.get_nodes();
  bool changed = false;
  for (const Nodes& nodes : after) {
    changed = changed || !Contains(before, nodes);
  }
  assert(changed);
  AssertFixedSize(after);
}

void TestGetNodesFiltersCollisionsWithoutDiscardingStoredPaths() {
  Maho maho = MakeMaho(0, 0, {{0.0, 0.0}}, 0.1);

  assert(maho.get_nodes().empty());

  maho.update_pose({10.0, 0.0, 0.0});

  AssertFixedSize(maho.get_nodes());
}

void TestDetectsGoalReached() {
  const Maho maho = MakeMaho();
  constexpr double kTwoPi = 6.28318530717958647692;

  assert(maho.is_goal_reached({2.9, 1.0, 0.2 + kTwoPi},
                              {{0.05, 0.0, 0.05}}));
  assert(!maho.is_goal_reached({2.9, 1.0, 0.2},
                               {{0.2, 0.0, 0.05}}));
  assert(!maho.is_goal_reached({2.7, 1.0, 0.2},
                               {{0.0, 0.0, 0.0}}));
}

void TestRejectsInvalidDt() {
  const EvaluationFunction evaluation_function = MakeEvaluationFunction();
  bool threw = false;
  try {
    const Maho maho(
        {{0.0, 0.0, 0.0},
         {{0.0, 0.0, 0.0}},
         {},
         {1.0, 0.0, 0.0},
         0.0,
         0,
         0,
         {0.1, 0.1, 0.1, 0.1}},
        Expander(ExpanderParams{{0.1, 0.1, 0.1}}),
        CollisionDetector({0.1}),
        Selector({1.0, 1.0, 1.0}, evaluation_function),
        Optimizer({0.1, 1e-4, {1.0, 1.0, 1.0}, {1.0, 1.0, 1.0}},
                  evaluation_function));
  } catch (const std::invalid_argument&) {
    threw = true;
  }
  assert(threw);
}

}  // namespace

int main() {
  TestConstructsFixedLengthNodeSequences();
  TestReplanAppendsThenDropsExpansionWhenOptimizationIsDisabled();
  TestUpdatePosePreservesNodesWhenOptimizationIsDisabled();
  TestUpdatePoseRunsConfiguredOptimizationCount();
  TestReplanRunsConfiguredOptimizationCount();
  TestGetNodesFiltersCollisionsWithoutDiscardingStoredPaths();
  TestDetectsGoalReached();
  TestRejectsInvalidDt();
}
