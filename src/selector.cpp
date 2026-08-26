#include "maho/selector.hpp"

#include <cmath>

namespace {

double AngleDifference(double lhs, double rhs) {
  return std::remainder(lhs - rhs, 2.0 * std::acos(-1.0));
}

}  // namespace

Selector::Selector(const SelectorParams& params,
                   const EvaluationFunction& evaluation_function)
    : params_(params), evaluation_function_(evaluation_function) {}

double Selector::terminalDistance(const Nodes& lhs, const Nodes& rhs,
                                  const Pose2D& initial_pose, double dt) const {
  const Pose2D lhs_pose =
      CalculateTerminalPose(initial_pose, lhs, dt);
  const Pose2D rhs_pose =
      CalculateTerminalPose(initial_pose, rhs, dt);
  const Node& lhs_node = lhs.back();
  const Node& rhs_node = rhs.back();
  const double pose_distance = std::sqrt(
      std::pow(lhs_pose.x - rhs_pose.x, 2) +
      std::pow(lhs_pose.y - rhs_pose.y, 2) +
      std::pow(AngleDifference(lhs_pose.theta, rhs_pose.theta), 2));
  const double velocity_distance = std::sqrt(
      std::pow(lhs_node.velocity.x - rhs_node.velocity.x, 2) +
      std::pow(lhs_node.velocity.y - rhs_node.velocity.y, 2) +
      std::pow(lhs_node.velocity.theta - rhs_node.velocity.theta, 2));
  return params_.pose_coefficient * pose_distance +
         params_.velocity_coefficient * velocity_distance;
}
