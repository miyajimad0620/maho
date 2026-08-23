#include "maho/maho.hpp"

#include <cassert>
#include <cmath>
#include <cstddef>

namespace {

Maho MakeMaho() {
  const MahoParams params{
      {{0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}},
      {},
      {20.0, 0.0, 0.0},
      {0.2, 0.1, 0.1, 0.1},
  };
  const Expander expander({1.0, {1.0, 1.0, 1.0}});
  const EvaluationFunction evaluation_function(
      {1.0, 1.0, 0.1, 1.0, 1.0, 1.0});
  const Evaluator evaluator(evaluation_function);
  const Selector selector({1.0, 1.0, 1.0});
  const Optimizer optimizer({
      0.1,
      1e-4,
      0,
      1.0,
      {100.0, 100.0, 100.0},
      {100.0, 100.0, 100.0},
  }, evaluation_function);
  return Maho(params, expander, evaluator, selector, optimizer);
}

void AssertSorted(const Maho::Paths& paths) {
  for (std::size_t i = 1; i < paths.size(); ++i) {
    assert(paths[i - 1].cost <= paths[i].cost);
  }
}

bool IsSameNode(const Node& lhs, const Node& rhs) {
  constexpr double kTolerance = 1e-12;
  return std::abs(lhs.pose.x - rhs.pose.x) < kTolerance &&
         std::abs(lhs.pose.y - rhs.pose.y) < kTolerance &&
         std::abs(lhs.pose.theta - rhs.pose.theta) < kTolerance &&
         std::abs(lhs.twist.x - rhs.twist.x) < kTolerance &&
         std::abs(lhs.twist.y - rhs.twist.y) < kTolerance &&
         std::abs(lhs.twist.theta - rhs.twist.theta) < kTolerance;
}

void AdvanceNode(Node* node, double dt) {
  const double cos_theta = std::cos(node->pose.theta);
  const double sin_theta = std::sin(node->pose.theta);
  node->pose.x +=
      (cos_theta * node->twist.x - sin_theta * node->twist.y) * dt;
  node->pose.y +=
      (sin_theta * node->twist.x + cos_theta * node->twist.y) * dt;
  node->pose.theta += node->twist.theta * dt;
}

void TestConstructsFixedLengthPaths() {
  const Maho maho = MakeMaho();
  const Maho::Paths& paths = maho.get_paths();

  assert(paths.size() == Maho::kPathCount);
  for (const Path& path : paths) {
    assert(path.nodes.size() == Maho::kNodeCount);
    assert(std::isfinite(path.cost));
  }
  AssertSorted(paths);
}

void TestUpdatesInitialNodeAndReevaluates() {
  Maho maho = MakeMaho();
  const Node initial_node{{2.0, 3.0, 0.5}, {0.5, -0.5, 0.25}};

  maho.update_init_node(initial_node);

  const Maho::Paths& paths = maho.get_paths();
  for (const Path& path : paths) {
    assert(path.nodes.front().pose.x == initial_node.pose.x);
    assert(path.nodes.front().pose.y == initial_node.pose.y);
    assert(path.nodes.front().pose.theta == initial_node.pose.theta);
    assert(path.nodes.front().twist.x == initial_node.twist.x);
    assert(std::isfinite(path.cost));
  }
  AssertSorted(paths);
}

void TestReplansTerminalNodes() {
  Maho maho = MakeMaho();
  const Maho::Paths before = maho.get_paths();
  const Expander expander({1.0, {1.0, 1.0, 1.0}});

  maho.replan();

  const Maho::Paths& after = maho.get_paths();
  for (const Path& path : after) {
    assert(path.nodes.size() == Maho::kNodeCount);
    assert(std::isfinite(path.cost));
  }
  AssertSorted(after);

  for (const Path& replanned : after) {
    bool is_expanded_from_previous_path = false;
    for (const Path& previous : before) {
      bool prefix_matches = true;
      for (std::size_t i = 0; i + 2 < replanned.nodes.size(); ++i) {
        prefix_matches =
            prefix_matches &&
            IsSameNode(replanned.nodes[i], previous.nodes[i + 1]);
      }
      if (!prefix_matches) {
        continue;
      }

      const Expander::ExpandedNodes expanded_nodes =
          expander.expand(previous.nodes.back());
      for (const Node& expanded_node : expanded_nodes) {
        is_expanded_from_previous_path =
            is_expanded_from_previous_path ||
            IsSameNode(replanned.nodes.back(), expanded_node);
      }
    }
    assert(is_expanded_from_previous_path);
  }
}

void TestReplansFromCurrentNode() {
  Maho maho = MakeMaho();
  const Node current{{1.5, -0.5, 0.25}, {0.3, -0.2, 0.1}};

  maho.replan(current);

  for (const Path& path : maho.get_paths()) {
    assert(path.nodes.size() == Maho::kNodeCount);
    assert(IsSameNode(path.nodes.front(), current));
    for (std::size_t i = 1; i < path.nodes.size(); ++i) {
      const Node& previous = path.nodes[i - 1];
      const Node& node = path.nodes[i];
      const double cos_theta = std::cos(previous.pose.theta);
      const double sin_theta = std::sin(previous.pose.theta);
      assert(std::abs(node.pose.x -
                      (previous.pose.x +
                       (cos_theta * node.twist.x -
                        sin_theta * node.twist.y))) < 1e-9);
      assert(std::abs(node.pose.y -
                      (previous.pose.y +
                       (sin_theta * node.twist.x +
                        cos_theta * node.twist.y))) < 1e-9);
      assert(std::abs(node.pose.theta -
                      (previous.pose.theta + node.twist.theta)) < 1e-9);
    }
  }
}

void TestDetectsGoalReached() {
  const Maho maho = MakeMaho();
  constexpr double kTwoPi = 6.28318530717958647692;
  assert(maho.is_goal_reached(
      {{19.9, 0.0, kTwoPi}, {0.05, 0.0, 0.05}}));
  assert(!maho.is_goal_reached(
      {{19.9, 0.0, kTwoPi}, {0.2, 0.0, 0.05}}));
  assert(!maho.is_goal_reached(
      {{19.7, 0.0, kTwoPi}, {0.0, 0.0, 0.0}}));
}

void TestReachesGoalAndStops() {
  constexpr double kDt = 0.2;
  const MahoParams params{
      {{0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}},
      {},
      {4.0, 0.0, 0.0},
      {0.1, 0.1, 0.1, 0.1},
  };
  const Expander expander({kDt, {0.2, 0.2, 0.15}});
  const EvaluationFunction evaluation_function(
      {0.0, 0.0, 0.1, 5.0, 0.2, 0.1,
       {20.0, 1.0, 2.0, 1.0, 0.1, 5.0, 1.0, 1.0, 1.0, 1.5}});
  const Evaluator evaluator(evaluation_function);
  const Selector selector({1.0, 0.5, 0.2});
  const Optimizer optimizer({
      0.02,
      1e-4,
      0,
      kDt,
      {2.0, 2.0, 1.5},
      {0.2, 0.2, 0.15},
  }, evaluation_function);
  Maho maho(params, expander, evaluator, selector, optimizer);
  Node current = params.initial_node;

  for (std::size_t step = 0;
       step < 60 && !maho.is_goal_reached(current); ++step) {
    current.twist = maho.get_paths().front().nodes[1].twist;
    AdvanceNode(&current, kDt);
    maho.replan(current);
  }

  assert(maho.is_goal_reached(current));
}

}  // namespace

int main() {
  TestConstructsFixedLengthPaths();
  TestUpdatesInitialNodeAndReevaluates();
  TestReplansTerminalNodes();
  TestReplansFromCurrentNode();
  TestDetectsGoalReached();
  TestReachesGoalAndStops();
}
