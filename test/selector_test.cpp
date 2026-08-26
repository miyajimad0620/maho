#include "maho/selector.hpp"

#include <array>
#include <cassert>
#include <cmath>
#include <limits>

namespace {

EvaluatedNodes MakeEvaluatedNodes(double velocity, double angular_velocity,
                                  double cost) {
  return {{{{velocity, 0.0, angular_velocity}}}, cost};
}

}  // namespace

int main() {
  const Selector selector({1.0, 1.0, 0.1});
  const Pose2D initial_pose{0.0, 0.0, 0.0};
  const std::array<EvaluatedNodes, 5> candidates{{
      MakeEvaluatedNodes(0.0, 0.0, 1.0),
      MakeEvaluatedNodes(0.01, 0.01, 2.0),
      MakeEvaluatedNodes(10.0, 0.0, 3.0),
      MakeEvaluatedNodes(20.0, 0.0, 100.0),
      MakeEvaluatedNodes(30.0, 0.0,
                         std::numeric_limits<double>::infinity()),
  }};

  const auto selected = selector.select<2>(candidates, initial_pose, 1.0);
  assert(selected.size() == 2);
  assert(selected[0].cost == 1.0);
  assert(selected[1].cost == 3.0);

  const std::array<EvaluatedNodes, 2> insufficient{{
      MakeEvaluatedNodes(1.0, 0.0, 4.0),
      MakeEvaluatedNodes(2.0, 0.0,
                         std::numeric_limits<double>::infinity()),
  }};
  const auto padded = selector.select<3>(insufficient, initial_pose, 1.0);
  assert(padded[0].cost == 4.0);
  assert(std::isinf(padded[1].cost));
  assert(std::isinf(padded[2].cost));
}
