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
      candidates, {}, 1.0, 1.0, {}, {});

  assert(selected.size() == 2);
  assert(selected[0].front().velocity.x == 0.0);
  assert(selected[1].front().velocity.x == 1.0);
}

void TestPadsInsufficientCandidates() {
  const Selector selector({1.0, 1.0, 1.0}, MakeEvaluationFunction());
  const std::array<Nodes, 2> candidates{{MakeNodes(1.0), {}}};

  const auto selected = selector.select<3>(
      candidates, {}, 1.0, 1.0, {}, {});

  assert(selected[0].front().velocity.x == 1.0);
  assert(selected[1].empty());
  assert(selected[2].empty());
}

void TestKeepsDistantCandidate() {
  const Selector selector(
      {1.0, 1.0, 0.0},
      EvaluationFunction({0.0, 0.0, 0.0, 0.0, 0.0, 0.0}));
  const std::array<Nodes, 3> candidates{{
      MakeNodes(0.0),
      MakeNodes(1.0),
      MakeNodes(10.0),
  }};

  const auto selected = selector.select<2>(
      candidates, {}, 1.0, 1.0, {}, {});

  bool contains_one = false;
  bool contains_ten = false;
  for (const Nodes& nodes : selected) {
    contains_one = contains_one || nodes.front().velocity.x == 1.0;
    contains_ten = contains_ten || nodes.front().velocity.x == 10.0;
  }
  assert(contains_one);
  assert(contains_ten);
}

}  // namespace

int main() {
  TestEvaluatesAndSelectsCandidates();
  TestPadsInsufficientCandidates();
  TestKeepsDistantCandidate();
}
