#include "maho/maho.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

Maho::Maho(const MahoParams& params, const Expander& expander,
           const Evaluator& evaluator, const Selector& selector,
           const Optimizer& optimizer)
    : current_pose_(params.initial_pose),
      env_(params.env),
      goal_(params.goal),
      dt_(params.dt),
      replan_optimization_count_(params.replan_optimization_count),
      pose_update_optimization_count_(params.pose_update_optimization_count),
      goal_reached_(params.goal_reached),
      expander_(expander),
      evaluator_(evaluator),
      selector_(selector),
      optimizer_(optimizer) {
  if (dt_ <= 0.0 || goal_reached_.position_tolerance < 0.0 ||
      goal_reached_.angle_tolerance < 0.0 ||
      goal_reached_.velocity_tolerance < 0.0 ||
      goal_reached_.angular_velocity_tolerance < 0.0) {
    throw std::invalid_argument("invalid maho parameter");
  }

  evaluated_nodes_[0].nodes.push_back(params.initial_node);
  evaluate(&evaluated_nodes_[0]);

  for (std::size_t node_count = 1; node_count < kNodeSequenceLength;
       node_count += Expander::kExpansionPathLength) {
    Candidates candidates = expandNodes();
    for (EvaluatedNodes& candidate : candidates) {
      if (candidate.nodes.empty()) {
        continue;
      }
      optimize(&candidate, replan_optimization_count_);
      if (candidate.nodes.size() > kNodeSequenceLength) {
        candidate.nodes.resize(kNodeSequenceLength);
        evaluate(&candidate);
      }
    }
    evaluated_nodes_ = selector_.select<kNodeSequenceCount>(
        candidates, current_pose_, dt_);
  }
}

void Maho::replan(const Pose2D& pose) {
  current_pose_ = pose;
  Candidates candidates = expandNodes();
  for (EvaluatedNodes& candidate : candidates) {
    if (candidate.nodes.empty()) {
      continue;
    }
    optimize(&candidate, replan_optimization_count_);
    candidate.nodes.resize(kNodeSequenceLength);
    evaluate(&candidate);
  }
  evaluated_nodes_ = selector_.select<kNodeSequenceCount>(
      candidates, current_pose_, dt_);
}

void Maho::update_pose(const Pose2D& pose) {
  current_pose_ = pose;
  for (EvaluatedNodes& evaluated_nodes : evaluated_nodes_) {
    optimize(&evaluated_nodes, pose_update_optimization_count_);
  }
  sortNodes();
}

bool Maho::is_goal_reached(const Pose2D& pose, const Node& node) const {
  constexpr double kTwoPi = 6.28318530717958647692;
  const double position_error =
      std::hypot(goal_.x - pose.x, goal_.y - pose.y);
  const double angle_error =
      std::abs(std::remainder(goal_.theta - pose.theta, kTwoPi));
  const double velocity =
      std::hypot(node.velocity.x, node.velocity.y);
  return position_error <= goal_reached_.position_tolerance &&
         angle_error <= goal_reached_.angle_tolerance &&
         velocity <= goal_reached_.velocity_tolerance &&
         std::abs(node.velocity.theta) <=
             goal_reached_.angular_velocity_tolerance;
}

Maho::NodeSequences Maho::get_nodes() const {
  NodeSequences nodes;
  for (std::size_t i = 0; i < nodes.size(); ++i) {
    nodes[i] = evaluated_nodes_[i].nodes;
  }
  return nodes;
}

Maho::Candidates Maho::expandNodes() const {
  Candidates candidates{};
  std::size_t candidate_index = 0;
  for (const EvaluatedNodes& evaluated_nodes : evaluated_nodes_) {
    if (evaluated_nodes.nodes.empty()) {
      continue;
    }

    const Expander::ExpandedPaths expanded_paths =
        expander_.expand(evaluated_nodes.nodes.back());
    for (const Expander::ExpansionPath& expanded_path : expanded_paths) {
      EvaluatedNodes& candidate = candidates[candidate_index++];
      candidate.nodes = evaluated_nodes.nodes;
      candidate.nodes.insert(candidate.nodes.end(), expanded_path.begin(),
                             expanded_path.end());
    }
  }
  return candidates;
}

void Maho::optimize(EvaluatedNodes* evaluated_nodes,
                    std::size_t optimization_count) const {
  if (evaluated_nodes->nodes.empty()) {
    return;
  }
  for (std::size_t i = 0; i < optimization_count; ++i) {
    evaluated_nodes->nodes = optimizer_.optimize(
        evaluated_nodes->nodes, current_pose_, dt_, env_, goal_);
  }
  evaluate(evaluated_nodes);
}

void Maho::evaluate(EvaluatedNodes* evaluated_nodes) const {
  evaluated_nodes->cost = evaluator_.evaluate(
      evaluated_nodes->nodes, current_pose_, dt_, env_, goal_);
}

void Maho::sortNodes() {
  std::sort(evaluated_nodes_.begin(), evaluated_nodes_.end(),
            [](const EvaluatedNodes& lhs, const EvaluatedNodes& rhs) {
              return lhs.cost < rhs.cost;
            });
}
