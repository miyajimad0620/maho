#include "maho/collision_detector.hpp"

#include <cassert>
#include <cmath>
#include <stdexcept>

namespace {

void TestDetectsCollisionAtIntegratedPose() {
  const CollisionDetector detector({0.5});
  const Nodes nodes{{{1.0, 0.0, 0.0}}, {{1.0, 0.0, 0.0}}};

  assert(detector.detectsCollision(nodes, {}, 1.0, 1.0,
                                   {{2.5, 0.0}}));
}

void TestReturnsFalseForCollisionFreePath() {
  const CollisionDetector detector({0.5});
  const Nodes nodes{{{1.0, 0.0, 0.0}}, {{1.0, 0.0, 0.0}}};

  assert(!detector.detectsCollision(nodes, {}, 1.0, 1.0,
                                    {{3.0, 0.0}}));
  assert(!detector.detectsCollision({}, {}, 1.0, 1.0,
                                    {{0.0, 0.0}}));
}

void TestDetectsCollisionBetweenNodeEndpoints() {
  const CollisionDetector detector({0.1});
  const Nodes nodes{{{2.0, 0.0, 0.0}}};

  assert(detector.detectsCollision(nodes, {}, 1.0, 1.0,
                                   {{1.0, 0.0}}));
}

void TestDetectsCollisionAlongArc() {
  constexpr double kHalfPi = 1.57079632679489661923;
  const CollisionDetector detector({0.01});
  const Nodes nodes{{{1.0, 0.0, kHalfPi}}};
  const Point2D midpoint{
      std::sin(kHalfPi / 2.0) / kHalfPi,
      (1.0 - std::cos(kHalfPi / 2.0)) / kHalfPi,
  };

  assert(detector.detectsCollision(nodes, {}, 1.0, 1.0, {midpoint}));
}

void TestDetectsCollisionAlongClockwiseArc() {
  constexpr double kHalfPi = 1.57079632679489661923;
  const CollisionDetector detector({0.01});
  const Nodes nodes{{{1.0, 0.0, -kHalfPi}}};
  const Point2D midpoint{
      std::sin(kHalfPi / 2.0) / kHalfPi,
      -(1.0 - std::cos(kHalfPi / 2.0)) / kHalfPi,
  };

  assert(detector.detectsCollision(nodes, {}, 1.0, 1.0, {midpoint}));
}

void TestIgnoresUnsweptPartOfCircle() {
  constexpr double kHalfPi = 1.57079632679489661923;
  const double radius = 1.0 / kHalfPi;
  const CollisionDetector detector({0.01});
  const Nodes nodes{{{1.0, 0.0, kHalfPi}}};

  assert(!detector.detectsCollision(
      nodes, {}, 1.0, 1.0, {{-radius, radius}}));
}

void TestUsesFirstDtForFirstTrajectory() {
  const CollisionDetector detector({0.01});
  const Nodes nodes{{{1.0, 0.0, 0.0}}, {{1.0, 0.0, 0.0}}};

  assert(detector.detectsCollision(nodes, {}, 1.0, 1.0,
                                   {{1.75, 0.0}}));
  assert(!detector.detectsCollision(nodes, {}, 1.0, 0.25,
                                    {{1.75, 0.0}}));
}

void TestAllowsZeroFirstDt() {
  const CollisionDetector detector({0.01});
  const Nodes nodes{{{1.0, 0.0, 0.0}}, {{0.0, 0.0, 0.0}}};

  assert(!detector.detectsCollision(nodes, {}, 1.0, 0.0,
                                    {{0.5, 0.0}}));
}

void TestRejectsNegativeRobotRadius() {
  bool threw = false;
  try {
    const CollisionDetector detector({-0.1});
  } catch (const std::invalid_argument&) {
    threw = true;
  }
  assert(threw);
}

void TestRejectsInvalidDt() {
  const CollisionDetector detector({0.5});
  bool threw = false;
  try {
    detector.detectsCollision({}, {}, 0.0, 0.0, {});
  } catch (const std::invalid_argument&) {
    threw = true;
  }
  assert(threw);
}

void TestRejectsInvalidFirstDt() {
  const CollisionDetector detector({0.5});
  for (const double first_dt : {-0.1, 1.1}) {
    bool threw = false;
    try {
      detector.detectsCollision({}, {}, 1.0, first_dt, {});
    } catch (const std::invalid_argument&) {
      threw = true;
    }
    assert(threw);
  }
}

}  // namespace

int main() {
  TestDetectsCollisionAtIntegratedPose();
  TestReturnsFalseForCollisionFreePath();
  TestDetectsCollisionBetweenNodeEndpoints();
  TestDetectsCollisionAlongArc();
  TestDetectsCollisionAlongClockwiseArc();
  TestIgnoresUnsweptPartOfCircle();
  TestUsesFirstDtForFirstTrajectory();
  TestAllowsZeroFirstDt();
  TestRejectsNegativeRobotRadius();
  TestRejectsInvalidDt();
  TestRejectsInvalidFirstDt();
}
