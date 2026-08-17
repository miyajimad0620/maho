#include "maho/evaluator.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

Evaluator::Evaluator(const EvaluatorParams& params) : params_(params) {}

double Evaluator::evaluate(const Nodes& nodes, const Env& env,
                           const Goal& goal) const {
  if (nodes.empty()) {
    return std::numeric_limits<double>::infinity();
  }

  double cost = 0.0;

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
  }

  const Node& last_node = nodes.back();
  const double dx = goal.x - last_node.pose.x;
  const double dy = goal.y - last_node.pose.y;
  constexpr double kTwoPi = 6.28318530717958647692;
  const double dtheta =
      std::remainder(goal.theta - last_node.pose.theta, kTwoPi);
  const double pose_difference = std::hypot(std::hypot(dx, dy), dtheta);

  return cost + params_.goal_cost_coefficient * pose_difference;
}
