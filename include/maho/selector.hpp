#ifndef MAHO__SELECTOR_HPP_
#define MAHO__SELECTOR_HPP_

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <limits>

#include "maho/env.hpp"
#include "maho/evaluation_function.hpp"
#include "maho/kinematics.hpp"
#include "maho/node.hpp"

struct SelectorParams {
  double position_coefficient;
  double cost_coefficient;
};

class Selector {
 public:
  Selector(const SelectorParams& params,
           const EvaluationFunction& evaluation_function);

  double evaluate(const Nodes& nodes, const Pose2D& initial_pose,
                  double dt, double first_dt, const Env& env,
                  const Goal& goal) const;

  template <std::size_t SelectedCount, std::size_t CandidateCount>
  std::array<Nodes, SelectedCount> select(
      const std::array<Nodes, CandidateCount>& candidates,
      const Pose2D& initial_pose, double dt, double first_dt,
      const Env& env, const Goal& goal) const {
    struct CandidateInfo {
      double cost = std::numeric_limits<double>::infinity();
      double min_distance = std::numeric_limits<double>::infinity();
      bool is_valid = false;
      bool is_selected = false;
    };
    std::array<CandidateInfo, CandidateCount> candidate_infos{};
    std::array<Nodes, SelectedCount> selected{};
    if constexpr (SelectedCount == 0) {
      return selected;
    }

    std::size_t valid_count = 0;
    std::size_t selected_index = CandidateCount;
    double min_cost = std::numeric_limits<double>::infinity();
    for (std::size_t i = 0; i < candidates.size(); ++i) {
      const Nodes& nodes = candidates[i];
      if (nodes.empty()) {
        continue;
      }
      const double cost = evaluate(nodes, initial_pose, dt, first_dt, env,
                                   goal);
      if (!std::isfinite(cost)) {
        continue;
      }
      candidate_infos[i].cost = cost;
      candidate_infos[i].is_valid = true;
      ++valid_count;
      if (cost < min_cost) {
        min_cost = cost;
        selected_index = i;
      }
    }

    if (valid_count == 0) {
      return selected;
    }

    std::array<std::size_t, SelectedCount> selected_indices{};
    std::size_t selected_count = 0;
    while (selected_count < SelectedCount && selected_count < valid_count) {
      const Nodes& selected_nodes = candidates[selected_index];
      candidate_infos[selected_index].is_selected = true;
      selected_indices[selected_count++] = selected_index;

      if (selected_count == SelectedCount || selected_count == valid_count) {
        break;
      }

      double min_priority = std::numeric_limits<double>::infinity();
      std::size_t next_index = CandidateCount;
      for (std::size_t j = 0; j < CandidateCount; ++j) {
        CandidateInfo& candidate_info = candidate_infos[j];
        if (!candidate_info.is_valid || candidate_info.is_selected) {
          continue;
        }

        const double distance = pathDistance(
            selected_nodes, candidates[j], initial_pose, dt, first_dt);
        candidate_info.min_distance =
            std::min(candidate_info.min_distance, distance);

        const double priority =
            params_.cost_coefficient * candidate_info.cost -
            candidate_info.min_distance;
        if (priority < min_priority) {
          min_priority = priority;
          next_index = j;
        }
      }
      selected_index = next_index;
    }

    std::sort(selected_indices.begin(),
              selected_indices.begin() + selected_count,
              [&candidate_infos](std::size_t lhs, std::size_t rhs) {
                return candidate_infos[lhs].cost <
                       candidate_infos[rhs].cost;
              });
    for (std::size_t i = 0; i < selected_count; ++i) {
      selected[i] = candidates[selected_indices[i]];
    }
    return selected;
  }

 private:
  double pathDistance(const Nodes& lhs, const Nodes& rhs,
                      const Pose2D& initial_pose, double dt,
                      double first_dt) const;

  SelectorParams params_;
  EvaluationFunction evaluation_function_;
};

#endif  // MAHO__SELECTOR_HPP_
