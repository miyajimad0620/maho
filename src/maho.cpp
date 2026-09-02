#include "maho/maho.hpp"

#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>

#include "maho/kinematics.hpp"

Maho::Maho(const MahoParams& params, const Expander& expander,
           const CollisionDetector& collision_detector,
           const Selector& selector,
           const Optimizer& optimizer)
    : current_pose_(params.initial_pose),
      initial_node_(params.initial_node),
      env_(params.env),
      goal_(params.goal),
      dt_(params.dt),
      first_dt_(params.dt),
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

  initialization_history_.push_back(get_node_sequences_with_status());
  while (node_sequences_[0].size() < kStoredNodeSequenceLength) {
    Candidates candidates = expandNodes(true);
    for (Nodes& candidate : candidates) {
      optimize(&candidate, replan_optimization_count_, current_pose_, dt_, 1);
    }
    StoredNodeSequences selected = selector_.select<kNodeSequenceCount>(
        candidates, current_pose_, dt_, first_dt_, env_, goal_);
    for (std::size_t i = 0; i < selected.size(); ++i) {
      selected[i].erase(selected[i].begin());
      node_sequences_[i] = std::move(selected[i]);
    }
    initialization_history_.push_back(get_node_sequences_with_status());
  }
}

void Maho::replan(const Pose2D& pose) {
  current_pose_ = pose;
  first_dt_ = dt_;
  Candidates candidates = expandNodes(false);
  for (Nodes& candidate : candidates) {
    optimize(&candidate, replan_optimization_count_, current_pose_, dt_);
  }

  std::size_t best_index = 0;
  double best_cost = std::numeric_limits<double>::infinity();
  for (std::size_t i = 0; i < candidates.size(); ++i) {
    const double cost = selector_.evaluate(
        candidates[i], current_pose_, dt_, dt_, env_, goal_);
    if (cost < best_cost) {
      best_cost = cost;
      best_index = i;
    }
  }

  const Node next_initial_node = candidates[best_index].front();
  for (std::size_t i = 0; i < candidates.size(); ++i) {
    if (i == best_index) {
      continue;
    }
    const Twist2D original = candidates[i].front().velocity;
    if (replan_optimization_count_ == 0) {
      candidates[i].front() = next_initial_node;
      continue;
    }
    for (std::size_t step = 0; step < replan_optimization_count_; ++step) {
      const double weight = static_cast<double>(step + 1) /
                            static_cast<double>(replan_optimization_count_);
      Twist2D& velocity = candidates[i].front().velocity;
      velocity.x = original.x +
                   (next_initial_node.velocity.x - original.x) * weight;
      velocity.y = original.y +
                   (next_initial_node.velocity.y - original.y) * weight;
      velocity.theta =
          original.theta +
          (next_initial_node.velocity.theta - original.theta) * weight;
      optimize(&candidates[i], 1, current_pose_, dt_, 1);
    }
  }

  StoredNodeSequences selected = selector_.select<kNodeSequenceCount>(
      candidates, current_pose_, dt_, first_dt_, env_, goal_);
  initial_node_ = next_initial_node;
  for (std::size_t i = 0; i < selected.size(); ++i) {
    selected[i].erase(selected[i].begin());
    node_sequences_[i] = std::move(selected[i]);
  }
}

void Maho::update_pose(const Pose2D& pose, double dt_replan) {
  if (!std::isfinite(dt_replan) || dt_replan < 0.0 || dt_replan > dt_) {
    throw std::invalid_argument("invalid replan elapsed time");
  }
  current_pose_ = pose;
  first_dt_ = dt_ - dt_replan;
  const Pose2D next_node_initial_pose =
      IntegratePose(current_pose_, initial_node_.velocity, first_dt_);
  for (Nodes& nodes : node_sequences_) {
    optimize(&nodes, pose_update_optimization_count_,
             next_node_initial_pose, dt_);
  }
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
  for (const NodeSequenceStatus& status :
       get_node_sequences_with_status()) {
    if (!status.collides) {
      nodes.push_back(status.nodes);
    }
  }
  return nodes;
}

Maho::NodeSequenceStatuses Maho::get_node_sequences_with_status() const {
  NodeSequenceStatuses statuses;
  statuses.reserve(kNodeSequenceCount);
  for (const Nodes& candidate : node_sequences_) {
    Nodes nodes;
    nodes.reserve(kNodeSequenceLength);
    nodes.push_back(initial_node_);
    nodes.insert(nodes.end(), candidate.begin(), candidate.end());
    statuses.push_back({
        nodes,
        collision_detector_.detectsCollision(
            nodes, current_pose_, dt_, first_dt_, env_),
    });
  }
  return statuses;
}

const Maho::InitializationHistory& Maho::get_initialization_history() const {
  return initialization_history_;
}

Maho::Candidates Maho::expandNodes(bool include_initial_node) const {
  Candidates candidates{};
  std::size_t candidate_index = 0;
  for (const Nodes& nodes : node_sequences_) {
    Nodes expansion_path = nodes;
    if (include_initial_node) {
      expansion_path.insert(expansion_path.begin(), initial_node_);
    }
    const Expander::ExpandedPaths expanded_paths =
        expander_.expand(expansion_path);
    for (const Nodes& expanded_path : expanded_paths) {
      Nodes& candidate = candidates[candidate_index++];
      candidate = expanded_path;
      if (include_initial_node) {
        candidate.front() = initial_node_;
      }
    }
  }
  return candidates;
}

void Maho::optimize(Nodes* nodes, std::size_t optimization_count,
                    const Pose2D& initial_pose, double first_dt,
                    std::size_t fixed_node_count) const {
  if (nodes->empty()) {
    return;
  }
  for (std::size_t i = 0; i < optimization_count; ++i) {
    *nodes = optimizer_.optimize(*nodes, initial_pose, dt_, first_dt, env_,
                                 goal_, fixed_node_count);
  }
}
