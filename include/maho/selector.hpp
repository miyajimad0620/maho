#ifndef MAHO__SELECTOR_HPP_
#define MAHO__SELECTOR_HPP_

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <limits>
#include <vector>

#include "maho/env.hpp"
#include "maho/evaluation_function.hpp"
#include "maho/kinematics.hpp"
#include "maho/node.hpp"

struct SelectorParams {
  double pose_coefficient;
  double velocity_coefficient;
  double cost_coefficient;
};

class Selector {
 public:
  Selector(const SelectorParams& params,
           const EvaluationFunction& evaluation_function);

  template <std::size_t SelectedCount, std::size_t CandidateCount>
  std::array<Nodes, SelectedCount> select(
      const std::array<Nodes, CandidateCount>& candidates,
      const Pose2D& initial_pose, double dt, const Env& env,
      const Goal& goal) const {
    struct ScoredNodes {
      Nodes nodes;
      double cost;
    };

    std::vector<ScoredNodes> remaining;
    remaining.reserve(CandidateCount);
    for (const Nodes& nodes : candidates) {
      if (nodes.empty()) {
        continue;
      }
      const double cost = evaluation_function_.evaluate(
          nodes, initial_pose, dt, env, goal);
      if (std::isfinite(cost)) {
        remaining.push_back({nodes, cost});
      }
    }

    while (remaining.size() > SelectedCount) {
      std::size_t removal_index = 0;
      double highest_priority = -std::numeric_limits<double>::infinity();
      for (std::size_t i = 0; i < remaining.size(); ++i) {
        double nearest_distance = std::numeric_limits<double>::infinity();
        for (std::size_t j = 0; j < remaining.size(); ++j) {
          if (i != j) {
            nearest_distance = std::min(
                nearest_distance,
                terminalDistance(remaining[i].nodes, remaining[j].nodes,
                                 initial_pose, dt));
          }
        }

        const double similarity = 1.0 / (1.0 + nearest_distance);
        const double priority =
            params_.cost_coefficient * remaining[i].cost + similarity;
        if (priority > highest_priority) {
          highest_priority = priority;
          removal_index = i;
        }
      }
      remaining.erase(remaining.begin() + removal_index);
    }

    std::sort(remaining.begin(), remaining.end(),
              [](const ScoredNodes& lhs, const ScoredNodes& rhs) {
                return lhs.cost < rhs.cost;
              });

    std::array<Nodes, SelectedCount> selected{};
    std::transform(remaining.begin(), remaining.end(), selected.begin(),
                   [](const ScoredNodes& scored_nodes) {
                     return scored_nodes.nodes;
                   });
    return selected;
  }

 private:
  double terminalDistance(const Nodes& lhs, const Nodes& rhs,
                          const Pose2D& initial_pose, double dt) const;

  SelectorParams params_;
  EvaluationFunction evaluation_function_;
};

#endif  // MAHO__SELECTOR_HPP_
