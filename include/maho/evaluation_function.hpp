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

enum class CollisionHandling {
  kReturnInfinity,
  kUseFiniteCost,
};

class EvaluationFunction {
 public:
  explicit EvaluationFunction(const EvaluationFunctionParams& params);

  double evaluate(const Nodes& nodes, const Pose2D& initial_pose, double dt,
                  const Env& env, const Goal& goal,
                  CollisionHandling collision_handling =
                      CollisionHandling::kReturnInfinity) const;

 private:
  EvaluationFunctionParams params_;
};

#endif  // MAHO__EVALUATION_FUNCTION_HPP_
