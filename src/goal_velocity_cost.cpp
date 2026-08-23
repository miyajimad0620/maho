#include "maho/goal_velocity_cost.hpp"

#include <algorithm>
#include <cmath>

namespace {

constexpr double kTwoPi = 6.28318530717958647692;

double AngularVelocityCost(const Node& node, const Pose2D& goal,
                           const GoalVelocityCostParams& params) {
  const double angle_error =
      std::remainder(goal.theta - node.pose.theta, kTwoPi);
  const double target_angular_velocity = std::clamp(
      angle_error * params.angular_distance_coefficient,
      -params.max_angular_velocity, params.max_angular_velocity);
  const double error = node.twist.theta - target_angular_velocity;
  return params.angular_velocity_cost_coefficient * error * error;
}

}  // namespace

double CalculateGoalVelocityCost(const Node& node, const Pose2D& goal,
                                 const GoalVelocityCostParams& params) {
  const double cos_theta = std::cos(node.pose.theta);
  const double sin_theta = std::sin(node.pose.theta);
  const double world_velocity_x =
      cos_theta * node.twist.x - sin_theta * node.twist.y;
  const double world_velocity_y =
      sin_theta * node.twist.x + cos_theta * node.twist.y;

  const double dx = goal.x - node.pose.x;
  const double dy = goal.y - node.pose.y;
  const double distance = std::hypot(dx, dy);
  if (distance <= params.position_tolerance) {
    return params.cost_coefficient *
           (world_velocity_x * world_velocity_x +
            world_velocity_y * world_velocity_y +
            AngularVelocityCost(node, goal, params));
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
          AngularVelocityCost(node, goal, params));
}

double CalculateTerminalGoalVelocityCost(
    const Node& node, const Pose2D& goal,
    const GoalVelocityCostParams& params) {
  const double distance =
      std::hypot(goal.x - node.pose.x, goal.y - node.pose.y);
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
              (node.twist.x * node.twist.x + node.twist.y * node.twist.y) +
          params.terminal_angular_velocity_cost_coefficient *
              node.twist.theta * node.twist.theta);
}
