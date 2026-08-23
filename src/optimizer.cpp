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

double ClampVelocity(double value, double previous, double max_change,
                     double max_velocity) {
  return std::clamp(ClampChange(value, previous, max_change), -max_velocity,
                    max_velocity);
}

}  // namespace

Optimizer::Optimizer(const OptimizerParams& params,
                     const EvaluationFunction& evaluation_function)
    : params_(params), evaluation_function_(evaluation_function) {
  if (params_.learning_rate <= 0.0 ||
      params_.finite_difference_step <= 0.0 || params_.dt <= 0.0 ||
      params_.max_velocity.x < 0.0 || params_.max_velocity.y < 0.0 ||
      params_.max_velocity.theta < 0.0 ||
      params_.max_velocity_change.x < 0.0 ||
      params_.max_velocity_change.y < 0.0 ||
      params_.max_velocity_change.theta < 0.0) {
    throw std::invalid_argument("invalid optimizer parameter");
  }
}

std::vector<Node> Optimizer::optimize(const std::vector<Node>& nodes,
                                      const Env& env,
                                      const Pose2D& goal) const {
  std::vector<Node> optimized = nodes;
  enforceKinematicConstraints(&optimized);
  if (nodes.size() < 2 || params_.max_iterations == 0) {
    return optimized;
  }

  for (std::size_t iteration = 0; iteration < params_.max_iterations;
       ++iteration) {
    std::vector<Node> next = optimized;
    const double current_cost = evaluation_function_.evaluate(
        optimized, env, goal, CollisionHandling::kUseFiniteCost);
    for (std::size_t i = 1; i < optimized.size(); ++i) {
      for (double Pose2D::*component : {&Pose2D::x, &Pose2D::y,
                                       &Pose2D::theta}) {
        std::vector<Node> lower = optimized;
        std::vector<Node> upper = optimized;
        lower[i].pose.*component -= params_.finite_difference_step;
        upper[i].pose.*component += params_.finite_difference_step;
        enforceKinematicConstraints(&lower);
        enforceKinematicConstraints(&upper);
        const double lower_cost = evaluation_function_.evaluate(
            lower, env, goal, CollisionHandling::kUseFiniteCost);
        const double upper_cost = evaluation_function_.evaluate(
            upper, env, goal, CollisionHandling::kUseFiniteCost);
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
    enforceKinematicConstraints(&next);
    optimized = std::move(next);
  }
  return optimized;
}

void Optimizer::enforceKinematicConstraints(std::vector<Node>* nodes) const {
  for (std::size_t i = 1; i < nodes->size(); ++i) {
    const Node& previous = (*nodes)[i - 1];
    Node& current = (*nodes)[i];
    const double cos_theta = std::cos(previous.pose.theta);
    const double sin_theta = std::sin(previous.pose.theta);
    const double dx = current.pose.x - previous.pose.x;
    const double dy = current.pose.y - previous.pose.y;
    const Twist2D desired{
        (cos_theta * dx + sin_theta * dy) / params_.dt,
        (-sin_theta * dx + cos_theta * dy) / params_.dt,
        AngleDifference(current.pose.theta, previous.pose.theta) / params_.dt,
    };
    current.twist.x =
        ClampVelocity(desired.x, previous.twist.x,
                      params_.max_velocity_change.x, params_.max_velocity.x);
    current.twist.y =
        ClampVelocity(desired.y, previous.twist.y,
                      params_.max_velocity_change.y, params_.max_velocity.y);
    current.twist.theta =
        ClampVelocity(desired.theta, previous.twist.theta,
                      params_.max_velocity_change.theta,
                      params_.max_velocity.theta);

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
