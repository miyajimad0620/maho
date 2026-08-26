#include "maho/maho.hpp"

#include <cmath>
#include <stdexcept>

Maho::Maho(const MahoParams& params, const Expander& expander,
           const CollisionDetector& collision_detector,
           const Selector& selector,
           const Optimizer& optimizer)
    : current_pose_(params.initial_pose),
      env_(params.env),
      goal_(params.goal),
      dt_(params.dt),
      replan_optimization_count_(params.replan_optimization_count),
      pose_update_optimization_count_(params.pose_update_optimization_count),
      goal_reached_(params.goal_reached),
      expander_(expander),
      collision_detector_(collision_detector),
      selector_(selector),
      optimizer_(optimizer) {
  if (dt_ <= 0.0 || goal_reached_.position_tolerance < 0.0 ||
      goal_reached_.angle_tolerance < 0.0 ||
      goal_reached_.velocity_tolerance < 0.0 ||
      goal_reached_.angular_velocity_tolerance < 0.0) {
    throw std::invalid_argument("invalid maho parameter");
  }

  node_sequences_[0].push_back(params.initial_node);

  for (std::size_t node_count = 1; node_count < kNodeSequenceLength;
       node_count += Expander::kExpansionPathLength) {
    Candidates candidates = expandNodes();
    for (Nodes& candidate : candidates) {
      if (candidate.empty()) {
        continue;
      }
      optimize(&candidate, replan_optimization_count_);
      if (candidate.size() > kNodeSequenceLength) {
        candidate.resize(kNodeSequenceLength);
      }
    }
    node_sequences_ = selector_.select<kNodeSequenceCount>(
        candidates, current_pose_, dt_, env_, goal_);
  }
}

void Maho::replan(const Pose2D& pose) {
  current_pose_ = pose;
  Candidates candidates = expandNodes();
  for (Nodes& candidate : candidates) {
    if (candidate.empty()) {
      continue;
    }
    optimize(&candidate, replan_optimization_count_);
    candidate.resize(kNodeSequenceLength);
  }
  node_sequences_ = selector_.select<kNodeSequenceCount>(
      candidates, current_pose_, dt_, env_, goal_);
}

void Maho::update_pose(const Pose2D& pose) {
  current_pose_ = pose;
  for (Nodes& nodes : node_sequences_) {
    optimize(&nodes, pose_update_optimization_count_);
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
  nodes.reserve(kNodeSequenceCount);
  for (const Nodes& candidate : node_sequences_) {
    if (!candidate.empty() &&
        !collision_detector_.detectsCollision(
            candidate, current_pose_, dt_, env_)) {
      nodes.push_back(candidate);
    }
  }
  return nodes;
}

Maho::Candidates Maho::expandNodes() const {
  Candidates candidates{};
  std::size_t candidate_index = 0;
  for (const Nodes& nodes : node_sequences_) {
    if (nodes.empty()) {
      continue;
    }

    const Expander::ExpandedPaths expanded_paths =
        expander_.expand(nodes.back());
    for (const Expander::ExpansionPath& expanded_path : expanded_paths) {
      Nodes& candidate = candidates[candidate_index++];
      candidate = nodes;
      candidate.insert(candidate.end(), expanded_path.begin(),
                       expanded_path.end());
    }
  }
  return candidates;
}

void Maho::optimize(Nodes* nodes, std::size_t optimization_count) const {
  if (nodes->empty()) {
    return;
  }
  for (std::size_t i = 0; i < optimization_count; ++i) {
    *nodes = optimizer_.optimize(*nodes, current_pose_, dt_, env_, goal_);
  }
}

void Maho::sortNodes() {
  node_sequences_ = selector_.select<kNodeSequenceCount>(
      node_sequences_, current_pose_, dt_, env_, goal_);
}
