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
  Node initial_node;
  Env env;
  Goal goal;
  GoalReachedParams goal_reached{};
};

class Maho {
 public:
  static constexpr std::size_t kPathCount = 10;
  static constexpr std::size_t kNodeCount = 20;
  using Paths = std::array<Path, kPathCount>;

  Maho(const MahoParams& params, const Expander& expander,
       const Evaluator& evaluator, const Selector& selector,
       const Optimizer& optimizer);

  void update_init_node(const Node& node);
  void replan();
  void replan(const Node& current_node);
  bool is_goal_reached(const Node& node) const;
  const Paths& get_paths() const;

 private:
  static constexpr std::size_t kCandidateCount =
      kPathCount * Expander::kExpansionCount;
  using CandidatePaths = std::array<Path, kCandidateCount>;

  CandidatePaths expandPaths(bool advance_path) const;
  void optimizeAndEvaluate(Path* path) const;
  void sortPaths();

  Env env_;
  Goal goal_;
  GoalReachedParams goal_reached_;
  Expander expander_;
  Evaluator evaluator_;
  Selector selector_;
  Optimizer optimizer_;
  Paths paths_{};
};

#endif  // MAHO__MAHO_HPP_
