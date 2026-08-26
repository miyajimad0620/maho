#include "maho/optimizer.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace {

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
      params_.finite_difference_step <= 0.0 ||
      params_.max_velocity.x < 0.0 || params_.max_velocity.y < 0.0 ||
      params_.max_velocity.theta < 0.0 ||
      params_.max_velocity_change.x < 0.0 ||
      params_.max_velocity_change.y < 0.0 ||
      params_.max_velocity_change.theta < 0.0) {
    throw std::invalid_argument("invalid optimizer parameter");
  }
}

Nodes Optimizer::optimize(const Nodes& nodes, const Pose2D& initial_pose,
                          double dt, const Env& env,
                          const Pose2D& goal) const {
  Nodes optimized = nodes;
  enforceVelocityConstraints(&optimized);
  if (nodes.empty()) {
    return optimized;
  }

  Nodes next = optimized;
  const double current_cost = evaluation_function_.evaluate(
      optimized, initial_pose, dt, env, goal);
  for (std::size_t i = 0; i < optimized.size(); ++i) {
    for (double Twist2D::*component : {&Twist2D::x, &Twist2D::y,
                                      &Twist2D::theta}) {
      Nodes lower = optimized;
      Nodes upper = optimized;
      lower[i].velocity.*component -= params_.finite_difference_step;
      upper[i].velocity.*component += params_.finite_difference_step;
      enforceVelocityConstraints(&lower);
      enforceVelocityConstraints(&upper);
      const double lower_cost = evaluation_function_.evaluate(
          lower, initial_pose, dt, env, goal);
      const double upper_cost = evaluation_function_.evaluate(
          upper, initial_pose, dt, env, goal);
      double gradient =
          (upper_cost - lower_cost) /
          (2.0 * params_.finite_difference_step);
      if (std::abs(gradient) < std::numeric_limits<double>::epsilon() &&
          upper_cost < current_cost) {
        // The distance cost has no unique gradient at an obstacle point.
        gradient =
            (upper_cost - current_cost) / params_.finite_difference_step;
      }
      next[i].velocity.*component -= params_.learning_rate * gradient;
    }
  }
  enforceVelocityConstraints(&next);
  return next;
}

void Optimizer::enforceVelocityConstraints(Nodes* nodes) const {
  for (std::size_t i = 0; i < nodes->size(); ++i) {
    Twist2D& velocity = (*nodes)[i].velocity;
    if (i == 0) {
      velocity.x = std::clamp(velocity.x, -params_.max_velocity.x,
                              params_.max_velocity.x);
      velocity.y = std::clamp(velocity.y, -params_.max_velocity.y,
                              params_.max_velocity.y);
      velocity.theta = std::clamp(velocity.theta,
                                  -params_.max_velocity.theta,
                                  params_.max_velocity.theta);
      continue;
    }

    const Twist2D& previous = (*nodes)[i - 1].velocity;
    velocity.x = ClampVelocity(velocity.x, previous.x,
                               params_.max_velocity_change.x,
                               params_.max_velocity.x);
    velocity.y = ClampVelocity(velocity.y, previous.y,
                               params_.max_velocity_change.y,
                               params_.max_velocity.y);
    velocity.theta = ClampVelocity(velocity.theta, previous.theta,
                                   params_.max_velocity_change.theta,
                                   params_.max_velocity.theta);
  }
}
