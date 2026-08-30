#include "maho/evaluation_function.hpp"

#include <algorithm>
#include <cstddef>
#include <cmath>
#include <limits>
#include <stdexcept>

#include "maho/kinematics.hpp"
#include "trajectory.hpp"

namespace {

constexpr double kTwoPi = 6.28318530717958647692;

double AngleDifference(double lhs, double rhs) {
  return std::remainder(lhs - rhs, kTwoPi);
}

}  // namespace

EvaluationFunction::EvaluationFunction(const EvaluationFunctionParams& params)
    : params_(params) {
  const GoalVelocityCostParams& velocity = params_.goal_velocity_cost;
  if (params_.obstacle_cost_coefficient < 0.0 ||
      params_.obstacle_influence_distance < 0.0 ||
      params_.robot_radius < 0.0 ||
      params_.goal_position_cost_coefficient < 0.0 ||
      params_.goal_angle_cost_coefficient < 0.0 ||
      params_.velocity_change_cost_coefficient < 0.0 ||
      velocity.cost_coefficient < 0.0 ||
      velocity.distance_coefficient < 0.0 || velocity.max_velocity < 0.0 ||
      velocity.lateral_velocity_cost_coefficient < 0.0 ||
      velocity.position_tolerance < 0.0 ||
      velocity.terminal_velocity_cost_coefficient < 0.0 ||
      velocity.terminal_angular_velocity_cost_coefficient < 0.0 ||
      velocity.angular_velocity_cost_coefficient < 0.0 ||
      velocity.angular_distance_coefficient < 0.0 ||
      velocity.max_angular_velocity < 0.0) {
    throw std::invalid_argument("invalid evaluation function parameter");
  }
}

double EvaluationFunction::evaluate(
    const Nodes& nodes, const Pose2D& initial_pose, double dt,
    double first_dt, const Env& env, const Goal& goal) const {
  if (!(dt > 0.0) || !(first_dt >= 0.0 && first_dt <= dt)) {
    throw std::invalid_argument("invalid evaluation dt");
  }
  if (nodes.empty()) {
    return std::numeric_limits<double>::infinity();
  }

  double total = 0.0;
  double goal_velocity_cost = 0.0;
  Pose2D pose = initial_pose;
  for (std::size_t i = 0; i < nodes.size(); ++i) {
    const Node& node = nodes[i];
    const double duration = i == 0 ? first_dt : dt;
    const Trajectory2D trajectory =
        CalculateTrajectory(pose, node.velocity, duration);
    double nearest_distance = std::numeric_limits<double>::infinity();
    for (const Point2D& point : env) {
      nearest_distance = std::min(nearest_distance,
                                  CalculateMinimumDistanceToTrajectory(
                                      trajectory, point));
    }

    if (std::isfinite(nearest_distance)) {
      const double clearance = nearest_distance - params_.robot_radius;
      const double penetration =
          std::max(0.0, params_.obstacle_influence_distance - clearance);
      total += params_.obstacle_cost_coefficient * penetration * penetration;
    }

    pose = trajectory.terminal_pose;
    goal_velocity_cost += CalculateGoalVelocityCost(
        node.velocity, pose, goal, params_.goal_velocity_cost);
  }
  total += goal_velocity_cost / static_cast<double>(nodes.size());

  for (std::size_t i = 1; i < nodes.size(); ++i) {
    const Twist2D& previous = nodes[i - 1].velocity;
    const Twist2D& current = nodes[i].velocity;
    const double dx = current.x - previous.x;
    const double dy = current.y - previous.y;
    const double dtheta = current.theta - previous.theta;
    total += params_.velocity_change_cost_coefficient *
             (dx * dx + dy * dy + dtheta * dtheta);
  }

  const Node& terminal_node = nodes.back();
  const double dx = pose.x - goal.x;
  const double dy = pose.y - goal.y;
  const double dtheta = AngleDifference(pose.theta, goal.theta);
  total += params_.goal_position_cost_coefficient * (dx * dx + dy * dy);
  total += params_.goal_angle_cost_coefficient * dtheta * dtheta;
  total += CalculateTerminalGoalVelocityCost(
      terminal_node.velocity, pose, goal, params_.goal_velocity_cost);
  return total;
}
