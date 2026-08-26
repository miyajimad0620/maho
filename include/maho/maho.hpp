#ifndef MAHO__MAHO_HPP_
#define MAHO__MAHO_HPP_

#include <array>
#include <cstddef>

#include "maho/env.hpp"
#include "maho/evaluator.hpp"
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
  using NodeSequences = std::array<Nodes, kNodeSequenceCount>;

  Maho(const MahoParams& params, const Expander& expander,
       const Evaluator& evaluator, const Selector& selector,
       const Optimizer& optimizer);

  void replan(const Pose2D& pose);
  void update_pose(const Pose2D& pose);
  bool is_goal_reached(const Pose2D& pose, const Node& node) const;
  NodeSequences get_nodes() const;

 private:
  static constexpr std::size_t kCandidateCount =
      kNodeSequenceCount * Expander::kExpansionCount;
  using EvaluatedNodeSequences =
      std::array<EvaluatedNodes, kNodeSequenceCount>;
  using Candidates = std::array<EvaluatedNodes, kCandidateCount>;

  Candidates expandNodes() const;
  void optimize(EvaluatedNodes* evaluated_nodes,
                std::size_t optimization_count) const;
  void evaluate(EvaluatedNodes* evaluated_nodes) const;
  void sortNodes();

  Pose2D current_pose_;
  Env env_;
  Goal goal_;
  double dt_;
  std::size_t replan_optimization_count_;
  std::size_t pose_update_optimization_count_;
  GoalReachedParams goal_reached_;
  Expander expander_;
  Evaluator evaluator_;
  Selector selector_;
  Optimizer optimizer_;
  EvaluatedNodeSequences evaluated_nodes_{};
};

#endif  // MAHO__MAHO_HPP_
