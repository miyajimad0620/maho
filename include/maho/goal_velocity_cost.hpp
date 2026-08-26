#ifndef MAHO__GOAL_VELOCITY_COST_HPP_
#define MAHO__GOAL_VELOCITY_COST_HPP_

#include "maho/node.hpp"

struct GoalVelocityCostParams {
  double cost_coefficient;
  double distance_coefficient;
  double max_velocity;
  double lateral_velocity_cost_coefficient;
  double position_tolerance;
  double terminal_velocity_cost_coefficient;
  double terminal_angular_velocity_cost_coefficient;
  double angular_velocity_cost_coefficient;
  double angular_distance_coefficient;
  double max_angular_velocity;
};

double CalculateGoalVelocityCost(const Twist2D& velocity,
                                 const Pose2D& pose, const Pose2D& goal,
                                 const GoalVelocityCostParams& params);
double CalculateTerminalGoalVelocityCost(
    const Twist2D& velocity, const Pose2D& pose, const Pose2D& goal,
    const GoalVelocityCostParams& params);

#endif  // MAHO__GOAL_VELOCITY_COST_HPP_
