#include "maho/selector.hpp"

#include <array>
#include <cassert>
#include <cmath>
#include <limits>

namespace {

Path MakePath(double x, double theta, double velocity, double cost) {
  return {{{{x, 0.0, theta}, {velocity, 0.0, 0.0}}}, cost};
}

}  // namespace

int main() {
  const Selector selector({1.0, 1.0, 0.1});
  const std::array<Path, 5> candidates{{
      MakePath(0.0, 0.0, 0.0, 1.0),
      MakePath(0.01, 0.01, 0.01, 2.0),
      MakePath(10.0, 0.0, 0.0, 3.0),
      MakePath(20.0, 0.0, 0.0, 100.0),
      MakePath(30.0, 0.0, 0.0, std::numeric_limits<double>::infinity()),
  }};

  const auto selected = selector.select<2>(candidates);
  assert(selected.size() == 2);
  assert(selected[0].cost == 1.0);
  assert(selected[1].cost == 3.0);

  const std::array<Path, 2> insufficient{{
      MakePath(1.0, 0.0, 0.0, 4.0),
      MakePath(2.0, 0.0, 0.0, std::numeric_limits<double>::infinity()),
  }};
  const auto padded = selector.select<3>(insufficient);
  assert(padded[0].cost == 4.0);
  assert(std::isinf(padded[1].cost));
  assert(std::isinf(padded[2].cost));
}
