#include "maho/goal_velocity_cost.hpp"

#include <algorithm>
#include <cmath>

namespace {

constexpr double kTwoPi = 6.28318530717958647692;

double AngularVelocityCost(const Twist2D& velocity, const Pose2D& pose,
                           const Pose2D& goal,
                           const GoalVelocityCostParams& params) {
  const double angle_error =
      std::remainder(goal.theta - pose.theta, kTwoPi);
  const double target_angular_velocity = std::clamp(
      angle_error * params.angular_distance_coefficient,
      -params.max_angular_velocity, params.max_angular_velocity);
  const double error = velocity.theta - target_angular_velocity;
  return params.angular_velocity_cost_coefficient * error * error;
}

}  // namespace

double CalculateGoalVelocityCost(const Twist2D& velocity,
                                 const Pose2D& pose, const Pose2D& goal,
                                 const GoalVelocityCostParams& params) {
  const double cos_theta = std::cos(pose.theta);
  const double sin_theta = std::sin(pose.theta);
  const double world_velocity_x =
      cos_theta * velocity.x - sin_theta * velocity.y;
  const double world_velocity_y =
      sin_theta * velocity.x + cos_theta * velocity.y;

  const double dx = goal.x - pose.x;
  const double dy = goal.y - pose.y;
  const double distance = std::hypot(dx, dy);
  if (distance <= params.position_tolerance) {
    return params.cost_coefficient *
           (world_velocity_x * world_velocity_x +
            world_velocity_y * world_velocity_y +
            AngularVelocityCost(velocity, pose, goal, params));
  }

  const double goal_direction_x = dx / distance;
  const double goal_direction_y = dy / distance;
  const double goal_direction_velocity =
      goal_direction_x * world_velocity_x +
      goal_direction_y * world_velocity_y;
  const double lateral_velocity =
      -goal_direction_y * world_velocity_x +
      goal_direction_x * world_velocity_y;
  const double target_velocity =
      std::min(distance * params.distance_coefficient, params.max_velocity);
  const double goal_direction_velocity_error =
      goal_direction_velocity - target_velocity;

  return params.cost_coefficient *
         (goal_direction_velocity_error * goal_direction_velocity_error +
          params.lateral_velocity_cost_coefficient * lateral_velocity *
              lateral_velocity +
          AngularVelocityCost(velocity, pose, goal, params));
}

double CalculateTerminalGoalVelocityCost(
    const Twist2D& velocity, const Pose2D& pose, const Pose2D& goal,
    const GoalVelocityCostParams& params) {
  const double distance =
      std::hypot(goal.x - pose.x, goal.y - pose.y);
  double activation = 0.0;
  if (params.distance_coefficient > 0.0) {
    activation = params.max_velocity / params.distance_coefficient;
  }

  double proximity = 0.0;
  if (distance <= params.position_tolerance) {
    proximity = 1.0;
  } else if (activation > params.position_tolerance &&
             distance < activation) {
    proximity = (activation - distance) /
                (activation - params.position_tolerance);
  }

  return proximity *
         (params.terminal_velocity_cost_coefficient *
              (velocity.x * velocity.x + velocity.y * velocity.y) +
          params.terminal_angular_velocity_cost_coefficient *
              velocity.theta * velocity.theta);
}
