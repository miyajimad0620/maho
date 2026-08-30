#ifndef MAHO__MAHO_HPP_
#define MAHO__MAHO_HPP_

#include <array>
#include <cstddef>
#include <vector>

#include "maho/collision_detector.hpp"
#include "maho/env.hpp"
#include "maho/expander.hpp"
#include "maho/optimizer.hpp"
#include "maho/selector.hpp"

struct GoalReachedParams {
  double position_tolerance;
  double angle_tolerance;
  double velocity_tolerance;
  double angular_velocity_tolerance;
};

struct MahoParams {
  Pose2D initial_pose;
  Node initial_node;
  Env env;
  Goal goal;
  double dt;
  std::size_t replan_optimization_count;
  std::size_t pose_update_optimization_count;
  GoalReachedParams goal_reached{};
};

class Maho {
 public:
  static constexpr std::size_t kNodeSequenceCount = 10;
  static constexpr std::size_t kNodeSequenceLength = 20;
  using NodeSequences = std::vector<Nodes>;
  struct NodeSequenceStatus {
    Nodes nodes;
    bool collides;
  };
  using NodeSequenceStatuses = std::vector<NodeSequenceStatus>;

  Maho(const MahoParams& params, const Expander& expander,
       const CollisionDetector& collision_detector, const Selector& selector,
       const Optimizer& optimizer);

  void replan(const Pose2D& pose);
  void update_pose(const Pose2D& pose, double dt_replan);
  bool is_goal_reached(const Pose2D& pose, const Node& node) const;
  NodeSequences get_nodes() const;
  NodeSequenceStatuses get_node_sequences_with_status() const;

 private:
  using StoredNodeSequences = std::array<Nodes, kNodeSequenceCount>;
  static constexpr std::size_t kStoredNodeSequenceLength =
      kNodeSequenceLength - 1;
  static constexpr std::size_t kCandidateCount =
      kNodeSequenceCount * Expander::kExpansionCount;
  using Candidates = std::array<Nodes, kCandidateCount>;

  Candidates expandNodes() const;
  void optimize(Nodes* nodes, std::size_t optimization_count,
                const Pose2D& initial_pose, double first_dt,
                std::size_t fixed_node_count = 0) const;

  Pose2D current_pose_;
  Node initial_node_;
  Env env_;
  Goal goal_;
  double dt_;
  double first_dt_;
  std::size_t replan_optimization_count_;
  std::size_t pose_update_optimization_count_;
  GoalReachedParams goal_reached_;
  Expander expander_;
  CollisionDetector collision_detector_;
  Selector selector_;
  Optimizer optimizer_;
  StoredNodeSequences node_sequences_{};
};

#endif  // MAHO__MAHO_HPP_
