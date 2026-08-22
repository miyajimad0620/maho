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
  };
  const Expander expander({1.0, {1.0, 1.0, 1.0}});
  const Evaluator evaluator({1.0, 0.1, 1.0});
  const Selector selector({1.0, 1.0, 1.0});
  const Optimizer optimizer({
      1.0,
      1.0,
      0.1,
      0.1,
      1.0,
      1.0,
      0.1,
      1e-4,
      0,
      1.0,
      {100.0, 100.0, 100.0},
      {100.0, 100.0, 100.0},
  });
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

bool IsSameTwist(const Twist2D& lhs, const Twist2D& rhs) {
  constexpr double kTolerance = 1e-12;
  return std::abs(lhs.x - rhs.x) < kTolerance &&
         std::abs(lhs.y - rhs.y) < kTolerance &&
         std::abs(lhs.theta - rhs.theta) < kTolerance;
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
            IsSameTwist(
                replanned.nodes[replanned.nodes.size() - 2].twist,
                expanded_node.twist);
      }
    }
    assert(is_expanded_from_previous_path);
  }
}

}  // namespace

int main() {
  TestConstructsFixedLengthPaths();
  TestUpdatesInitialNodeAndReevaluates();
  TestReplansTerminalNodes();
}
