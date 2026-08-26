#ifndef MAHO__SELECTOR_HPP_
#define MAHO__SELECTOR_HPP_

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <limits>
#include <vector>

#include "maho/kinematics.hpp"
#include "maho/node.hpp"

struct EvaluatedNodes {
  Nodes nodes;
  double cost = std::numeric_limits<double>::infinity();
};

struct SelectorParams {
  double pose_coefficient;
  double velocity_coefficient;
  double cost_coefficient;
};

class Selector {
 public:
  explicit Selector(const SelectorParams& params);

  template <std::size_t SelectedCount, std::size_t CandidateCount>
  std::array<EvaluatedNodes, SelectedCount> select(
      const std::array<EvaluatedNodes, CandidateCount>& candidates,
      const Pose2D& initial_pose, double dt) const {
    std::vector<EvaluatedNodes> remaining;
    remaining.reserve(CandidateCount);
    for (const EvaluatedNodes& evaluated_nodes : candidates) {
      if (!evaluated_nodes.nodes.empty() &&
          std::isfinite(evaluated_nodes.cost)) {
        remaining.push_back(evaluated_nodes);
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
                terminalDistance(remaining[i], remaining[j], initial_pose,
                                 dt));
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
              [](const EvaluatedNodes& lhs, const EvaluatedNodes& rhs) {
                return lhs.cost < rhs.cost;
              });

    std::array<EvaluatedNodes, SelectedCount> selected{};
    std::copy(remaining.begin(), remaining.end(), selected.begin());
    return selected;
  }

 private:
  double terminalDistance(const EvaluatedNodes& lhs,
                          const EvaluatedNodes& rhs,
                          const Pose2D& initial_pose, double dt) const;

  SelectorParams params_;
};

#endif  // MAHO__SELECTOR_HPP_
