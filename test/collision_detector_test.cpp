#include "maho/collision_detector.hpp"

#include <cassert>
#include <stdexcept>

namespace {

void TestDetectsCollisionAtIntegratedPose() {
  const CollisionDetector detector({0.5});
  const Nodes nodes{{{1.0, 0.0, 0.0}}, {{1.0, 0.0, 0.0}}};

  assert(detector.detectsCollision(nodes, {}, 1.0, {{2.5, 0.0}}));
}

void TestReturnsFalseForCollisionFreePath() {
  const CollisionDetector detector({0.5});
  const Nodes nodes{{{1.0, 0.0, 0.0}}, {{1.0, 0.0, 0.0}}};

  assert(!detector.detectsCollision(nodes, {}, 1.0, {{3.0, 0.0}}));
  assert(!detector.detectsCollision({}, {}, 1.0, {{0.0, 0.0}}));
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
    detector.detectsCollision({}, {}, 0.0, {});
  } catch (const std::invalid_argument&) {
    threw = true;
  }
  assert(threw);
}

}  // namespace

int main() {
  TestDetectsCollisionAtIntegratedPose();
  TestReturnsFalseForCollisionFreePath();
  TestRejectsNegativeRobotRadius();
  TestRejectsInvalidDt();
}
