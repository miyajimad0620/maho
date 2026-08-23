#ifndef MAHO__EVALUATOR_HPP_
#define MAHO__EVALUATOR_HPP_

#include <vector>

#include "maho/env.hpp"
#include "maho/goal_velocity_cost.hpp"
#include "maho/node.hpp"

using Nodes = std::vector<Node>;
using Goal = Pose2D;

struct EvaluatorParams {
  double obstacle_cost_coefficient;
  double robot_radius;
  double goal_cost_coefficient;
  GoalVelocityCostParams goal_velocity_cost{};
};

class Evaluator {
 public:
  explicit Evaluator(const EvaluatorParams& params);

  double evaluate(const Nodes& nodes, const Env& env, const Goal& goal) const;

 private:
  EvaluatorParams params_;
};

#endif  // MAHO__EVALUATOR_HPP_
