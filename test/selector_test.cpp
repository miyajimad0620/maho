#include "maho/selector.hpp"

#include <array>
#include <cassert>
#include <limits>

namespace {

EvaluationFunction MakeEvaluationFunction() {
  return EvaluationFunction({0.0, 0.0, 0.0, 1.0, 0.0, 0.0});
}

Nodes MakeNodes(double velocity) {
  return {{{velocity, 0.0, 0.0}}};
}

void TestEvaluatesAndSelectsCandidates() {
  const Selector selector({1.0, 1.0}, MakeEvaluationFunction());
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
  const Selector selector({1.0, 1.0}, MakeEvaluationFunction());
  const std::array<Nodes, 2> candidates{{MakeNodes(1.0), {}}};

  const auto selected = selector.select<3>(
      candidates, {}, 1.0, 1.0, {}, {});

  assert(selected[0].front().velocity.x == 1.0);
  assert(selected[1].empty());
  assert(selected[2].empty());
}

void TestKeepsDistantCandidate() {
  const Selector selector(
      {1.0, 0.0},
      EvaluationFunction({0.0, 0.0, 0.0, 0.0, 0.0, 0.0}));
  const std::array<Nodes, 3> candidates{{
      MakeNodes(0.0),
      MakeNodes(1.0),
      MakeNodes(10.0),
  }};

  const auto selected = selector.select<2>(
      candidates, {}, 1.0, 1.0, {}, {});

  bool contains_zero = false;
  bool contains_ten = false;
  for (const Nodes& nodes : selected) {
    contains_zero = contains_zero || nodes.front().velocity.x == 0.0;
    contains_ten = contains_ten || nodes.front().velocity.x == 10.0;
  }
  assert(contains_zero);
  assert(contains_ten);
}

void TestDoesNotPreferDuplicateCandidate() {
  const Selector selector(
      {1.0, 0.0},
      EvaluationFunction({0.0, 0.0, 0.0, 0.0, 0.0, 0.0}));
  const std::array<Nodes, 3> candidates{{
      MakeNodes(0.0),
      MakeNodes(0.0),
      MakeNodes(10.0),
  }};

  const auto selected = selector.select<2>(
      candidates, {}, 1.0, 1.0, {}, {});

  assert(selected[0].front().velocity.x == 0.0);
  assert(selected[1].front().velocity.x == 10.0);
}

void TestUsesIntermediatePathPositions() {
  const Selector selector(
      {1.0, 0.0},
      EvaluationFunction({0.0, 0.0, 0.0, 0.0, 0.0, 0.0}));
  const Nodes forward_then_back{
      {{1.0, 0.0, 0.0}},
      {{-1.0, 0.0, 0.0}},
      {{0.0, 0.0, 0.0}},
  };
  const Nodes backward_then_forward{
      {{-1.0, 0.0, 0.0}},
      {{1.0, 0.0, 0.0}},
      {{0.0, 0.0, 0.0}},
  };
  const std::array<Nodes, 3> candidates{{
      forward_then_back,
      forward_then_back,
      backward_then_forward,
  }};

  const auto selected = selector.select<2>(
      candidates, {}, 1.0, 1.0, {}, {});

  assert(selected[0].front().velocity.x == 1.0);
  assert(selected[1].front().velocity.x == -1.0);
}

void TestDoesNotUseVelocityWithoutPositionDifference() {
  const Selector selector(
      {1.0, 0.0},
      EvaluationFunction({0.0, 0.0, 0.0, 0.0, 0.0, 0.0}));
  const std::array<Nodes, 3> candidates{{
      {{{0.0, 0.0, 1.0}}},
      {{{0.0, 0.0, -1.0}}},
      MakeNodes(10.0),
  }};

  const auto selected = selector.select<2>(
      candidates, {}, 1.0, 1.0, {}, {});

  assert(selected[0].front().velocity.theta == 1.0);
  assert(selected[1].front().velocity.x == 10.0);
}

void TestIgnoresNonFiniteCandidates() {
  const Selector selector({1.0, 1.0}, MakeEvaluationFunction());
  const std::array<Nodes, 2> candidates{{
      MakeNodes(std::numeric_limits<double>::infinity()),
      MakeNodes(1.0),
  }};

  const auto selected = selector.select<2>(
      candidates, {}, 1.0, 1.0, {}, {});

  assert(selected[0].front().velocity.x == 1.0);
  assert(selected[1].empty());
}

void TestReturnsEmptySelectionWithoutValidCandidates() {
  const Selector selector({1.0, 1.0}, MakeEvaluationFunction());
  const std::array<Nodes, 2> candidates{};

  const auto selected = selector.select<2>(
      candidates, {}, 1.0, 1.0, {}, {});

  assert(selected[0].empty());
  assert(selected[1].empty());
}

void TestAllowsZeroSelectedCount() {
  const Selector selector({1.0, 1.0}, MakeEvaluationFunction());
  const std::array<Nodes, 1> candidates{{MakeNodes(1.0)}};

  const auto selected = selector.select<0>(
      candidates, {}, 1.0, 1.0, {}, {});

  assert(selected.empty());
}

}  // namespace

int main() {
  TestEvaluatesAndSelectsCandidates();
  TestPadsInsufficientCandidates();
  TestKeepsDistantCandidate();
  TestDoesNotPreferDuplicateCandidate();
  TestUsesIntermediatePathPositions();
  TestDoesNotUseVelocityWithoutPositionDifference();
  TestIgnoresNonFiniteCandidates();
  TestReturnsEmptySelectionWithoutValidCandidates();
  TestAllowsZeroSelectedCount();
}
