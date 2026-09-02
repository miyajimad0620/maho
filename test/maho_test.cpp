#include "maho/maho.hpp"

#include "maho/kinematics.hpp"

#include <cassert>
#include <cmath>
#include <cstddef>
#include <limits>
#include <stdexcept>

namespace {

constexpr double kTolerance = 1e-9;

EvaluationFunction MakeEvaluationFunction() {
  return EvaluationFunction({0.0, 0.0, 0.1, 1.0, 0.1, 0.05});
}

Optimizer MakeOptimizer() {
  return Optimizer({
      0.02,
      1e-4,
      {2.0, 2.0, 1.0},
      {0.2, 0.2, 0.1},
  }, MakeEvaluationFunction());
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
  const Selector selector({1.0, 0.2}, evaluation_function);
  const Optimizer optimizer = MakeOptimizer();
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

void AssertFixedSize(const Maho::NodeSequences& node_sequences) {
  assert(node_sequences.size() == Maho::kNodeSequenceCount);
  for (const Nodes& nodes : node_sequences) {
    assert(nodes.size() == Maho::kNodeSequenceLength);
  }
}

void AssertSortedForPose(const Maho::NodeSequences& node_sequences,
                         const Pose2D& pose, double first_dt = 0.2) {
  const EvaluationFunction evaluation_function = MakeEvaluationFunction();
  double previous_cost = -1.0;
  for (const Nodes& nodes : node_sequences) {
    const double cost = evaluation_function.evaluate(
        nodes, pose, 0.2, first_dt, {}, {3.0, 1.0, 0.2});
    assert(previous_cost <= cost + kTolerance);
    previous_cost = cost;
  }
}

void TestConstructsFixedLengthNodeSequences() {
  const Maho maho = MakeMaho();
  const Maho::NodeSequences node_sequences = maho.get_nodes();
  const Maho::InitializationHistory& initialization_history =
      maho.get_initialization_history();

  AssertFixedSize(node_sequences);
  AssertSortedForPose(node_sequences, {0.0, 0.0, 0.0});
  assert(initialization_history.size() == Maho::kNodeSequenceLength);
  for (std::size_t step = 0; step < initialization_history.size(); ++step) {
    const Maho::NodeSequenceStatuses& statuses =
        initialization_history[step];
    assert(statuses.size() == Maho::kNodeSequenceCount);
    for (const Maho::NodeSequenceStatus& status : statuses) {
      assert(status.nodes.size() == step + 1);
      assert(IsSameNode(status.nodes.front(),
                        statuses.front().nodes.front()));
    }
  }
  for (std::size_t path = 0; path < node_sequences.size(); ++path) {
    assert(IsSameNodes(initialization_history.back()[path].nodes,
                       node_sequences[path]));
  }
}

void TestReplanShiftsAndExpandsNodeSequences() {
  Maho maho = MakeMaho();
  const Maho::NodeSequences before = maho.get_nodes();
  const Pose2D pose{0.5, -0.2, 0.1};

  maho.replan(pose);

  const Maho::NodeSequences after = maho.get_nodes();
  AssertFixedSize(after);
  bool initial_node_came_from_previous_paths = false;
  for (const Nodes& parent : before) {
    initial_node_came_from_previous_paths =
        initial_node_came_from_previous_paths ||
        IsSameNode(after.front().front(), parent[1]);
  }
  assert(initial_node_came_from_previous_paths);
  for (const Nodes& nodes : after) {
    assert(IsSameNode(nodes.front(), after.front().front()));
    bool has_shifted_parent = false;
    for (const Nodes& parent : before) {
      bool same_prefix = true;
      for (std::size_t i = 1;
           i < Maho::kNodeSequenceLength -
                   Expander::kExpansionPathLength;
           ++i) {
        same_prefix = same_prefix && IsSameNode(nodes[i], parent[i + 1]);
      }
      has_shifted_parent = has_shifted_parent || same_prefix;
    }
    assert(has_shifted_parent);
  }
  AssertSortedForPose(after, pose);
}

void TestUpdatePosePreservesOrderWhenOptimizationIsDisabled() {
  Maho maho = MakeMaho();
  const Maho::NodeSequences before = maho.get_nodes();
  const Pose2D pose{1.0, 0.5, -0.2};

  maho.update_pose(pose, 0.1);

  const Maho::NodeSequences after = maho.get_nodes();
  assert(after.size() == before.size());
  for (std::size_t i = 0; i < after.size(); ++i) {
    assert(IsSameNodes(after[i], before[i]));
  }
}

void TestUpdatePoseOptimizesOnlyAfterFirstNode() {
  Maho maho = MakeMaho(0, 1);
  const Maho::NodeSequences before = maho.get_nodes();
  const Pose2D pose{0.0, 0.0, 0.0};
  const Pose2D next_node_initial_pose =
      IntegratePose(pose, before.front().front().velocity, 0.1);
  const Nodes first_internal_nodes(before.front().begin() + 1,
                                   before.front().end());
  const Nodes expected_first_internal_nodes = MakeOptimizer().optimize(
      first_internal_nodes, next_node_initial_pose, 0.2, 0.2, {},
      {3.0, 1.0, 0.2});

  maho.update_pose(pose, 0.1);

  const Maho::NodeSequences after = maho.get_nodes();
  bool changed = false;
  assert(after.size() == before.size());
  for (std::size_t path = 0; path < after.size(); ++path) {
    assert(IsSameNode(after[path].front(), before[path].front()));
    for (std::size_t node = 1; node < after[path].size(); ++node) {
      changed = changed ||
                !IsSameNode(after[path][node], before[path][node]);
    }
  }
  assert(changed);
  for (std::size_t node = 1; node < after.front().size(); ++node) {
    assert(IsSameNode(after.front()[node],
                      expected_first_internal_nodes[node - 1]));
  }
}

void TestGetNodesFiltersCollisionsWithoutDiscardingStoredPaths() {
  Maho maho = MakeMaho(0, 0, {{0.0, 0.0}}, 0.1);

  const Maho::NodeSequenceStatuses colliding_statuses =
      maho.get_node_sequences_with_status();
  assert(colliding_statuses.size() == Maho::kNodeSequenceCount);
  for (const Maho::NodeSequenceStatus& status : colliding_statuses) {
    assert(status.nodes.size() == Maho::kNodeSequenceLength);
    assert(status.collides);
  }
  assert(maho.get_nodes().empty());

  maho.update_pose({10.0, 0.0, 0.0}, 0.1);

  const Maho::NodeSequenceStatuses collision_free_statuses =
      maho.get_node_sequences_with_status();
  assert(collision_free_statuses.size() == Maho::kNodeSequenceCount);
  for (const Maho::NodeSequenceStatus& status : collision_free_statuses) {
    assert(!status.collides);
  }
  AssertFixedSize(maho.get_nodes());
}

void TestRejectsInvalidReplanElapsedTime() {
  Maho maho = MakeMaho();
  for (double dt_replan :
       {-0.1, 0.21, std::numeric_limits<double>::infinity()}) {
    bool threw = false;
    try {
      maho.update_pose({}, dt_replan);
    } catch (const std::invalid_argument&) {
      threw = true;
    }
    assert(threw);
  }
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
        Selector({1.0, 1.0}, evaluation_function),
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
  TestReplanShiftsAndExpandsNodeSequences();
  TestUpdatePosePreservesOrderWhenOptimizationIsDisabled();
  TestUpdatePoseOptimizesOnlyAfterFirstNode();
  TestGetNodesFiltersCollisionsWithoutDiscardingStoredPaths();
  TestRejectsInvalidReplanElapsedTime();
  TestDetectsGoalReached();
  TestRejectsInvalidDt();
}
