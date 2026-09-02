#ifndef MAHO__EVALUATION_FUNCTION_HPP_
#define MAHO__EVALUATION_FUNCTION_HPP_

#include "maho/env.hpp"
#include "maho/goal_velocity_cost.hpp"
#include "maho/node.hpp"

using Goal = Pose2D;

struct EvaluationFunctionParams {
  double obstacle_cost_coefficient;
  double obstacle_influence_distance;
  double robot_radius;
  double goal_position_cost_coefficient;
  double goal_angle_cost_coefficient;
  double velocity_change_cost_coefficient;
  GoalVelocityCostParams goal_velocity_cost{};
};

struct EvaluationResult {
  double cost;
  Nodes gradient;
};

class EvaluationFunction {
 public:
  explicit EvaluationFunction(const EvaluationFunctionParams& params);

  double evaluate(const Nodes& nodes, const Pose2D& initial_pose, double dt,
                  double first_dt, const Env& env, const Goal& goal) const;
  Nodes evaluate_grad(const Nodes& nodes, const Pose2D& initial_pose,
                      double dt, double first_dt, const Env& env,
                      const Goal& goal) const;
  EvaluationResult evaluate_with_grad(
      const Nodes& nodes, const Pose2D& initial_pose, double dt,
      double first_dt, const Env& env, const Goal& goal) const;

 private:
  EvaluationFunctionParams params_;
};

#endif  // MAHO__EVALUATION_FUNCTION_HPP_
