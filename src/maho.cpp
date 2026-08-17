#include "maho/maho.hpp"

#include <algorithm>
#include <utility>

Maho::Maho(const MahoParams& params, const Expander& expander,
           const Evaluator& evaluator, const Selector& selector,
           const Optimizer& optimizer)
    : env_(params.env),
      goal_(params.goal),
      expander_(expander),
      evaluator_(evaluator),
      selector_(selector),
      optimizer_(optimizer) {
  paths_[0].nodes.push_back(params.initial_node);
  paths_[0].cost = evaluator_.evaluate(paths_[0].nodes, env_, goal_);

  for (std::size_t node_count = 1; node_count < kNodeCount; ++node_count) {
    CandidatePaths candidates = expandPaths(false);
    paths_ = selector_.select<kPathCount>(candidates);
    for (Path& path : paths_) {
      optimizeAndEvaluate(&path);
    }
    sortPaths();
  }
}

void Maho::update_init_node(const Node& node) {
  for (Path& path : paths_) {
    if (path.nodes.empty()) {
      continue;
    }
    path.nodes.front() = node;
    optimizeAndEvaluate(&path);
  }
  sortPaths();
}

void Maho::replan() {
  CandidatePaths candidates = expandPaths(true);
  for (Path& candidate : candidates) {
    optimizeAndEvaluate(&candidate);
  }
  paths_ = selector_.select<kPathCount>(candidates);
  sortPaths();
}

const Maho::Paths& Maho::get_paths() const { return paths_; }

Maho::CandidatePaths Maho::expandPaths(bool replace_terminal) const {
  CandidatePaths candidates{};
  std::size_t candidate_index = 0;

  for (const Path& path : paths_) {
    if (path.nodes.empty() || (replace_terminal && path.nodes.size() < 2)) {
      continue;
    }

    const std::size_t expansion_index =
        replace_terminal ? path.nodes.size() - 2 : path.nodes.size() - 1;
    const Expander::ExpandedNodes expanded_nodes =
        expander_.expand(path.nodes[expansion_index]);
    for (const Node& expanded_node : expanded_nodes) {
      Path candidate = path;
      if (replace_terminal) {
        candidate.nodes.back() = expanded_node;
      } else {
        candidate.nodes.push_back(expanded_node);
      }
      candidate.cost = evaluator_.evaluate(candidate.nodes, env_, goal_);
      candidates[candidate_index++] = std::move(candidate);
    }
  }

  return candidates;
}

void Maho::optimizeAndEvaluate(Path* path) const {
  if (path->nodes.empty()) {
    return;
  }
  path->nodes = optimizer_.optimize(path->nodes, env_, goal_);
  path->cost = evaluator_.evaluate(path->nodes, env_, goal_);
}

void Maho::sortPaths() {
  std::sort(paths_.begin(), paths_.end(),
            [](const Path& lhs, const Path& rhs) {
              return lhs.cost < rhs.cost;
            });
}
