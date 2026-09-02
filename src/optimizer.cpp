#include "maho/optimizer.hpp"

#include <algorithm>
#include <cmath>
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
                          double dt, double first_dt, const Env& env,
                          const Pose2D& goal,
                          std::size_t fixed_node_count) const {
  if (!(dt > 0.0) || !(first_dt >= 0.0 && first_dt <= dt)) {
    throw std::invalid_argument("invalid optimization dt");
  }
  if (fixed_node_count > nodes.size()) {
    throw std::invalid_argument("invalid fixed node count");
  }

  Nodes optimized = nodes;
  enforceVelocityConstraints(&optimized, fixed_node_count);
  if (nodes.empty()) {
    return optimized;
  }

  Nodes next = optimized;
  const Nodes gradient = evaluation_function_.evaluate_grad(
      optimized, initial_pose, dt, first_dt, env, goal);
  for (std::size_t i = fixed_node_count; i < optimized.size(); ++i) {
    next[i].velocity.x -=
        params_.learning_rate * gradient[i].velocity.x;
    next[i].velocity.y -=
        params_.learning_rate * gradient[i].velocity.y;
    next[i].velocity.theta -=
        params_.learning_rate * gradient[i].velocity.theta;
  }
  enforceVelocityConstraints(&next, fixed_node_count);
  return next;
}

void Optimizer::enforceVelocityConstraints(
    Nodes* nodes, std::size_t fixed_node_count) const {
  for (std::size_t i = fixed_node_count; i < nodes->size(); ++i) {
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
