#include "maho/selector.hpp"

#include <cmath>

namespace {

double AngleDifference(double lhs, double rhs) {
  return std::remainder(lhs - rhs, 2.0 * std::acos(-1.0));
}

}  // namespace

Selector::Selector(const SelectorParams& params) : params_(params) {}

double Selector::terminalDistance(const Path& lhs, const Path& rhs) const {
  const Node& lhs_node = lhs.nodes.back();
  const Node& rhs_node = rhs.nodes.back();
  const double pose_distance = std::sqrt(
      std::pow(lhs_node.pose.x - rhs_node.pose.x, 2) +
      std::pow(lhs_node.pose.y - rhs_node.pose.y, 2) +
      std::pow(AngleDifference(lhs_node.pose.theta, rhs_node.pose.theta), 2));
  const double velocity_distance = std::sqrt(
      std::pow(lhs_node.twist.x - rhs_node.twist.x, 2) +
      std::pow(lhs_node.twist.y - rhs_node.twist.y, 2) +
      std::pow(lhs_node.twist.theta - rhs_node.twist.theta, 2));
  return params_.pose_coefficient * pose_distance +
         params_.velocity_coefficient * velocity_distance;
}
