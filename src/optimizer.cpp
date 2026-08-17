#include "maho/optimizer.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>

namespace {

constexpr double kTwoPi = 2.0 * 3.14159265358979323846;

double AngleDifference(double lhs, double rhs) {
  return std::remainder(lhs - rhs, kTwoPi);
}

double ClampChange(double value, double previous, double max_change) {
  return std::clamp(value, previous - max_change, previous + max_change);
}

}  // namespace

Optimizer::Optimizer(const OptimizerParams& params) : params_(params) {
  if (params_.obstacle_cost_coefficient < 0.0 ||
      params_.obstacle_influence_distance < 0.0 ||
      params_.robot_radius < 0.0 ||
      params_.goal_position_cost_coefficient < 0.0 ||
      params_.goal_angle_cost_coefficient < 0.0 ||
      params_.learning_rate <= 0.0 ||
      params_.finite_difference_step <= 0.0 || params_.dt <= 0.0 ||
      params_.max_velocity_change.x < 0.0 ||
      params_.max_velocity_change.y < 0.0 ||
      params_.max_velocity_change.theta < 0.0) {
    throw std::invalid_argument("invalid optimizer parameter");
  }
}

std::vector<Node> Optimizer::optimize(const std::vector<Node>& nodes,
                                      const Env& env,
                                      const Pose2D& goal) const {
  if (nodes.size() < 2 || params_.max_iterations == 0) {
    std::vector<Node> optimized = nodes;
    enforceVelocityChange(&optimized);
    return optimized;
  }

  std::vector<Node> optimized = nodes;
  for (std::size_t iteration = 0; iteration < params_.max_iterations;
       ++iteration) {
    std::vector<Node> next = optimized;
    const double current_cost = cost(optimized, env, goal);
    for (std::size_t i = 1; i < optimized.size(); ++i) {
      for (double Pose2D::*component : {&Pose2D::x, &Pose2D::y,
                                       &Pose2D::theta}) {
        std::vector<Node> lower = optimized;
        std::vector<Node> upper = optimized;
        lower[i].pose.*component -= params_.finite_difference_step;
        upper[i].pose.*component += params_.finite_difference_step;
        const double lower_cost = cost(lower, env, goal);
        const double upper_cost = cost(upper, env, goal);
        double gradient =
            (upper_cost - lower_cost) /
            (2.0 * params_.finite_difference_step);
        if (std::abs(gradient) < std::numeric_limits<double>::epsilon() &&
            upper_cost < current_cost) {
          // At an obstacle point the distance cost has no unique gradient, so
          // choose the positive direction when it decreases the total cost.
          gradient = (upper_cost - current_cost) /
                     params_.finite_difference_step;
        }
        next[i].pose.*component -= params_.learning_rate * gradient;
      }
    }
    enforceVelocityChange(&next);
    optimized = std::move(next);
  }
  return optimized;
}

double Optimizer::cost(const std::vector<Node>& nodes, const Env& env,
                       const Pose2D& goal) const {
  double total = 0.0;
  for (const Node& node : nodes) {
    double nearest_distance = std::numeric_limits<double>::infinity();
    for (const Point2D& point : env) {
      nearest_distance = std::min(
          nearest_distance,
          std::hypot(node.pose.x - point.x, node.pose.y - point.y));
    }

    if (std::isfinite(nearest_distance)) {
      const double clearance = nearest_distance - params_.robot_radius;
      const double penetration =
          std::max(0.0, params_.obstacle_influence_distance - clearance);
      total += params_.obstacle_cost_coefficient * penetration * penetration;
    }
  }

  const Pose2D& terminal = nodes.back().pose;
  const double dx = terminal.x - goal.x;
  const double dy = terminal.y - goal.y;
  const double dtheta = AngleDifference(terminal.theta, goal.theta);
  total += params_.goal_position_cost_coefficient * (dx * dx + dy * dy);
  total += params_.goal_angle_cost_coefficient * dtheta * dtheta;
  return total;
}

void Optimizer::enforceVelocityChange(std::vector<Node>* nodes) const {
  for (std::size_t i = 1; i < nodes->size(); ++i) {
    const Node& previous = (*nodes)[i - 1];
    Node& current = (*nodes)[i];
    const double dx = current.pose.x - previous.pose.x;
    const double dy = current.pose.y - previous.pose.y;
    const double cos_theta = std::cos(previous.pose.theta);
    const double sin_theta = std::sin(previous.pose.theta);
    const Twist2D desired{
        (cos_theta * dx + sin_theta * dy) / params_.dt,
        (-sin_theta * dx + cos_theta * dy) / params_.dt,
        AngleDifference(current.pose.theta, previous.pose.theta) / params_.dt,
    };

    current.twist.x =
        ClampChange(desired.x, previous.twist.x,
                    params_.max_velocity_change.x);
    current.twist.y =
        ClampChange(desired.y, previous.twist.y,
                    params_.max_velocity_change.y);
    current.twist.theta =
        ClampChange(desired.theta, previous.twist.theta,
                    params_.max_velocity_change.theta);

    current.pose.x =
        previous.pose.x +
        (cos_theta * current.twist.x - sin_theta * current.twist.y) *
            params_.dt;
    current.pose.y =
        previous.pose.y +
        (sin_theta * current.twist.x + cos_theta * current.twist.y) *
            params_.dt;
    current.pose.theta =
        previous.pose.theta + current.twist.theta * params_.dt;
  }
}
