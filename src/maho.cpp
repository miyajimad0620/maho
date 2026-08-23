#include "maho/maho.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <utility>

Maho::Maho(const MahoParams& params, const Expander& expander,
           const Evaluator& evaluator, const Selector& selector,
           const Optimizer& optimizer)
    : env_(params.env),
      goal_(params.goal),
      goal_reached_(params.goal_reached),
      expander_(expander),
      evaluator_(evaluator),
      selector_(selector),
      optimizer_(optimizer) {
  if (goal_reached_.position_tolerance < 0.0 ||
      goal_reached_.angle_tolerance < 0.0 ||
      goal_reached_.velocity_tolerance < 0.0 ||
      goal_reached_.angular_velocity_tolerance < 0.0) {
    throw std::invalid_argument("invalid goal reached parameter");
  }

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

void Maho::replan(const Node& current_node) {
  CandidatePaths candidates{};
  std::size_t candidate_index = 0;
  const bool near_goal =
      std::hypot(goal_.x - current_node.pose.x,
                 goal_.y - current_node.pose.y) <=
      goal_reached_.position_tolerance;
  for (const Path& path : paths_) {
    if (path.nodes.size() < 2) {
      continue;
    }

    Path shifted = path;
    shifted.nodes.erase(shifted.nodes.begin());
    shifted.nodes.front() = current_node;
    if (near_goal) {
      const Expander::ExpandedNodes immediate_nodes =
          expander_.expand(current_node);
      const Node terminal_node =
          expander_.expand(shifted.nodes.back())
              [Expander::kNoVelocityChangeIndex];
      for (const Node& immediate_node : immediate_nodes) {
        Path candidate = shifted;
        candidate.nodes[1] = immediate_node;
        candidate.nodes.push_back(terminal_node);
        candidates[candidate_index++] = std::move(candidate);
      }
    } else {
      const Expander::ExpandedNodes terminal_nodes =
          expander_.expand(shifted.nodes.back());
      for (const Node& terminal_node : terminal_nodes) {
        Path candidate = shifted;
        candidate.nodes.push_back(terminal_node);
        candidates[candidate_index++] = std::move(candidate);
      }
    }
  }

  for (Path& candidate : candidates) {
    optimizeAndEvaluate(&candidate);
  }
  paths_ = selector_.select<kPathCount>(candidates);
  sortPaths();
}

bool Maho::is_goal_reached(const Node& node) const {
  constexpr double kTwoPi = 6.28318530717958647692;
  const double position_error =
      std::hypot(goal_.x - node.pose.x, goal_.y - node.pose.y);
  const double angle_error =
      std::abs(std::remainder(goal_.theta - node.pose.theta, kTwoPi));
  const double velocity = std::hypot(node.twist.x, node.twist.y);
  return position_error <= goal_reached_.position_tolerance &&
         angle_error <= goal_reached_.angle_tolerance &&
         velocity <= goal_reached_.velocity_tolerance &&
         std::abs(node.twist.theta) <=
             goal_reached_.angular_velocity_tolerance;
}

const Maho::Paths& Maho::get_paths() const { return paths_; }

Maho::CandidatePaths Maho::expandPaths(bool advance_path) const {
  CandidatePaths candidates{};
  std::size_t candidate_index = 0;

  for (const Path& path : paths_) {
    if (path.nodes.empty()) {
      continue;
    }

    const Expander::ExpandedNodes expanded_nodes =
        expander_.expand(path.nodes.back());
    for (const Node& expanded_node : expanded_nodes) {
      Path candidate = path;
      if (advance_path) {
        candidate.nodes.erase(candidate.nodes.begin());
      }
      candidate.nodes.push_back(expanded_node);
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
