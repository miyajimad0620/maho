#include "maho/selector.hpp"

#include <array>
#include <cassert>

namespace {

EvaluationFunction MakeEvaluationFunction() {
  return EvaluationFunction({0.0, 0.0, 0.0, 1.0, 0.0, 0.0});
}

Nodes MakeNodes(double velocity) {
  return {{{velocity, 0.0, 0.0}}};
}

void TestEvaluatesAndSelectsCandidates() {
  const Selector selector({1.0, 1.0, 1.0}, MakeEvaluationFunction());
  const std::array<Nodes, 5> candidates{{
      MakeNodes(2.0),
      MakeNodes(0.0),
      MakeNodes(1.0),
      MakeNodes(3.0),
      {},
  }};

  const auto selected = selector.select<2>(
      candidates, {}, 1.0, {}, {});

  assert(selected.size() == 2);
  assert(selected[0].front().velocity.x == 0.0);
  assert(selected[1].front().velocity.x == 1.0);
}

void TestPadsInsufficientCandidates() {
  const Selector selector({1.0, 1.0, 1.0}, MakeEvaluationFunction());
  const std::array<Nodes, 2> candidates{{MakeNodes(1.0), {}}};

  const auto selected = selector.select<3>(
      candidates, {}, 1.0, {}, {});

  assert(selected[0].front().velocity.x == 1.0);
  assert(selected[1].empty());
  assert(selected[2].empty());
}

}  // namespace

int main() {
  TestEvaluatesAndSelectsCandidates();
  TestPadsInsufficientCandidates();
}
