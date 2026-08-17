#include "maho/evaluator.hpp"

#include <cassert>
#include <cmath>
#include <limits>

namespace {

bool IsNear(double actual, double expected) {
  return std::abs(actual - expected) < 1e-12;
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
}
