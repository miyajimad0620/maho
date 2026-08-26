#include "maho/expander.hpp"

#include "maho/kinematics.hpp"

#include <cassert>
#include <cmath>
#include <cstddef>

namespace {

constexpr double kTolerance = 1e-12;

bool IsNear(double lhs, double rhs) {
  return std::abs(lhs - rhs) < kTolerance;
}

bool IsSameNode(const Node& lhs, const Node& rhs) {
  return IsNear(lhs.velocity.x, rhs.velocity.x) &&
         IsNear(lhs.velocity.y, rhs.velocity.y) &&
         IsNear(lhs.velocity.theta, rhs.velocity.theta);
}

void TestExpandsVelocityBranches() {
  const Expander expander(ExpanderParams{{0.2, 0.3, 0.4}});
  const Node node{{1.0, -2.0, 0.5}};
  const Expander::ExpandedPaths expanded = expander.expand(node);
  const std::array<Twist2D, Expander::kExpansionCount> expected{{
      {0.8, -2.0, 0.5},
      {1.2, -2.0, 0.5},
      {1.0, -2.3, 0.5},
      {1.0, -1.7, 0.5},
      {1.0, -2.0, 0.1},
      {1.0, -2.0, 0.9},
      {1.0, -2.0, 0.5},
      {0.8, -1.7, 0.1},
  }};

  for (std::size_t branch = 0; branch < expanded.size(); ++branch) {
    const Node expected_node{expected[branch]};
    for (const Node& expanded_node : expanded[branch]) {
      assert(IsSameNode(expanded_node, expected_node));
    }
  }
}

void TestBrakingStopsSmallVelocities() {
  const Expander expander(ExpanderParams{{0.2, 0.3, 0.4}});
  const Node node{{0.1, -0.2, 0.3}};
  const Expander::ExpandedPaths expanded = expander.expand(node);

  const Expander::ExpansionPath& braking =
      expanded.back();

  for (const Node& expanded_node : braking) {
    assert(IsSameNode(expanded_node, Node{{0.0, 0.0, 0.0}}));
  }
}

void TestIntegratesPose() {
  constexpr double kHalfPi = 1.57079632679489661923;
  const Pose2D pose{1.0, 2.0, kHalfPi};
  const Pose2D integrated =
      IntegratePose(pose, {2.0, 1.0, 0.5}, 0.2);

  assert(IsNear(integrated.x, 0.8));
  assert(IsNear(integrated.y, 2.4));
  assert(IsNear(integrated.theta, kHalfPi + 0.1));
}

void TestCalculatesTerminalPose() {
  const Nodes nodes{{{1.0, 0.0, 0.5}}, {{1.0, 0.0, 0.0}}};
  const Pose2D terminal =
      CalculateTerminalPose({0.0, 0.0, 0.0}, nodes, 1.0);

  assert(IsNear(terminal.x, 1.0 + std::cos(0.5)));
  assert(IsNear(terminal.y, std::sin(0.5)));
  assert(IsNear(terminal.theta, 0.5));
}

}  // namespace

int main() {
  TestExpandsVelocityBranches();
  TestBrakingStopsSmallVelocities();
  TestIntegratesPose();
  TestCalculatesTerminalPose();
}
