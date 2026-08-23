#include "maho/evaluator.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

Evaluator::Evaluator(const EvaluatorParams& params) : params_(params) {
  const GoalVelocityCostParams& velocity = params_.goal_velocity_cost;
  if (velocity.cost_coefficient < 0.0 ||
      velocity.distance_coefficient < 0.0 || velocity.max_velocity < 0.0 ||
      velocity.lateral_velocity_cost_coefficient < 0.0 ||
      velocity.position_tolerance < 0.0 ||
      velocity.terminal_velocity_cost_coefficient < 0.0 ||
      velocity.terminal_angular_velocity_cost_coefficient < 0.0 ||
      velocity.angular_velocity_cost_coefficient < 0.0 ||
      velocity.angular_distance_coefficient < 0.0 ||
      velocity.max_angular_velocity < 0.0) {
    throw std::invalid_argument("invalid evaluator parameter");
  }
}

double Evaluator::evaluate(const Nodes& nodes, const Env& env,
                           const Goal& goal) const {
  if (nodes.empty()) {
    return std::numeric_limits<double>::infinity();
  }

  double cost = 0.0;
  double goal_velocity_cost = 0.0;

  for (const Node& node : nodes) {
    double nearest_distance = std::numeric_limits<double>::infinity();

    for (const Point2D& point : env) {
      nearest_distance = std::min(
          nearest_distance,
          std::hypot(point.x - node.pose.x, point.y - node.pose.y));
    }

    if (nearest_distance <= params_.robot_radius) {
      return std::numeric_limits<double>::infinity();
    }

    cost += params_.obstacle_cost_coefficient /
            (nearest_distance - params_.robot_radius);
    goal_velocity_cost += CalculateGoalVelocityCost(
        node, goal, params_.goal_velocity_cost);
  }

  cost += goal_velocity_cost / static_cast<double>(nodes.size());

  const Node& last_node = nodes.back();
  const double dx = goal.x - last_node.pose.x;
  const double dy = goal.y - last_node.pose.y;
  constexpr double kTwoPi = 6.28318530717958647692;
  const double dtheta =
      std::remainder(goal.theta - last_node.pose.theta, kTwoPi);
  const double pose_difference = std::hypot(std::hypot(dx, dy), dtheta);

  return cost + params_.goal_cost_coefficient * pose_difference +
         CalculateTerminalGoalVelocityCost(last_node,
                                           goal,
                                           params_.goal_velocity_cost);
}
