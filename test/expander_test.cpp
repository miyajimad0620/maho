#include "maho/expander.hpp"

#include "maho/kinematics.hpp"

#include <cassert>
#include <cmath>
#include <cstddef>
#include <stdexcept>

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
  const Nodes nodes{{{5.0, 6.0, 7.0}}, {{4.0, 5.0, 6.0}},
                    {{3.0, 4.0, 5.0}}, {{2.0, 3.0, 4.0}},
                    {{1.5, 2.0, 3.0}}, {{1.0, -2.0, 0.5}}};
  const Expander::ExpandedPaths expanded = expander.expand(nodes);
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
    assert(expanded[branch].size() == nodes.size() + 1);
    assert(IsSameNode(expanded[branch][0], nodes[0]));
    assert(IsSameNode(expanded[branch][1], nodes[1]));
    for (std::size_t i = 2; i < expanded[branch].size(); ++i) {
      assert(IsSameNode(expanded[branch][i], Node{expected[branch]}));
    }
  }
}

void TestBrakingStopsSmallVelocities() {
  const Expander expander(ExpanderParams{{0.2, 0.3, 0.4}});
  const Expander::ExpandedPaths expanded =
      expander.expand({{{0.1, -0.2, 0.3}}});

  for (const Node& node : expanded.back()) {
    assert(IsSameNode(node, Node{{0.0, 0.0, 0.0}}));
  }
}

void TestRejectsEmptyPath() {
  const Expander expander(ExpanderParams{{0.2, 0.3, 0.4}});
  bool threw = false;
  try {
    (void)expander.expand({});
  } catch (const std::invalid_argument&) {
    threw = true;
  }
  assert(threw);
}

void TestIntegratesPose() {
  constexpr double kHalfPi = 1.57079632679489661923;
  const Pose2D pose{1.0, 2.0, kHalfPi};
  const Pose2D integrated =
      IntegratePose(pose, {2.0, 1.0, 0.5}, 0.2);
  const double integrated_cosine = std::sin(0.1) / 0.5;
  const double integrated_sine = (1.0 - std::cos(0.1)) / 0.5;
  const double body_x = 2.0 * integrated_cosine - integrated_sine;
  const double body_y = 2.0 * integrated_sine + integrated_cosine;

  assert(IsNear(integrated.x, pose.x - body_y));
  assert(IsNear(integrated.y, pose.y + body_x));
  assert(IsNear(integrated.theta, kHalfPi + 0.1));
}

void TestIntegratesPoseWithNearZeroAngularVelocity() {
  const Pose2D integrated =
      IntegratePose({1.0, 2.0, 0.0}, {2.0, 1.0, 1e-12}, 0.2);

  assert(IsNear(integrated.x, 1.4));
  assert(IsNear(integrated.y, 2.2));
  assert(IsNear(integrated.theta, 2e-13));
}

void TestCalculatesTerminalPose() {
  const Nodes nodes{{{1.0, 0.0, 0.5}}, {{1.0, 0.0, 0.0}}};
  const Pose2D terminal =
      CalculateTerminalPose({0.0, 0.0, 0.0}, nodes, 1.0, 1.0);

  assert(IsNear(terminal.x, 2.0 * std::sin(0.5) + std::cos(0.5)));
  assert(IsNear(terminal.y,
                2.0 * (1.0 - std::cos(0.5)) + std::sin(0.5)));
  assert(IsNear(terminal.theta, 0.5));
}

}  // namespace

int main() {
  TestExpandsVelocityBranches();
  TestBrakingStopsSmallVelocities();
  TestRejectsEmptyPath();
  TestIntegratesPose();
  TestIntegratesPoseWithNearZeroAngularVelocity();
  TestCalculatesTerminalPose();
}
