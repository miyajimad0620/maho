#include "maho/evaluation_function.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <limits>
#include <stdexcept>
#include <vector>

#include "maho/kinematics.hpp"
#include "trajectory.hpp"

namespace {

constexpr double kTwoPi = 6.28318530717958647692;
constexpr double kStraightAngleTolerance = 1e-8;
constexpr double kAngleComparisonTolerance = 1e-12;
constexpr std::size_t kLocalVariableCount = 6;

double AngleDifference(double lhs, double rhs) {
  return std::remainder(lhs - rhs, kTwoPi);
}

void ValidateDt(double dt, double first_dt) {
  if (!(dt > 0.0) || !(first_dt >= 0.0 && first_dt <= dt)) {
    throw std::invalid_argument("invalid evaluation dt");
  }
}

struct Dual {
  double value{};
  std::array<double, kLocalVariableCount> derivative{};

  Dual() = default;
  Dual(double value_in) : value(value_in) {}

  static Dual Variable(double value, std::size_t index) {
    Dual result(value);
    result.derivative[index] = 1.0;
    return result;
  }
};

Dual operator+(const Dual& lhs, const Dual& rhs) {
  Dual result(lhs.value + rhs.value);
  for (std::size_t i = 0; i < kLocalVariableCount; ++i) {
    result.derivative[i] = lhs.derivative[i] + rhs.derivative[i];
  }
  return result;
}

Dual operator-(const Dual& lhs, const Dual& rhs) {
  Dual result(lhs.value - rhs.value);
  for (std::size_t i = 0; i < kLocalVariableCount; ++i) {
    result.derivative[i] = lhs.derivative[i] - rhs.derivative[i];
  }
  return result;
}

Dual operator-(const Dual& value) {
  Dual result(-value.value);
  for (std::size_t i = 0; i < kLocalVariableCount; ++i) {
    result.derivative[i] = -value.derivative[i];
  }
  return result;
}

Dual operator*(const Dual& lhs, const Dual& rhs) {
  Dual result(lhs.value * rhs.value);
  for (std::size_t i = 0; i < kLocalVariableCount; ++i) {
    result.derivative[i] = lhs.derivative[i] * rhs.value +
                           lhs.value * rhs.derivative[i];
  }
  return result;
}

Dual operator/(const Dual& lhs, const Dual& rhs) {
  Dual result(lhs.value / rhs.value);
  const double denominator = rhs.value * rhs.value;
  for (std::size_t i = 0; i < kLocalVariableCount; ++i) {
    result.derivative[i] =
        (lhs.derivative[i] * rhs.value - lhs.value * rhs.derivative[i]) /
        denominator;
  }
  return result;
}

Dual Sin(const Dual& value) {
  Dual result(std::sin(value.value));
  const double scale = std::cos(value.value);
  for (std::size_t i = 0; i < kLocalVariableCount; ++i) {
    result.derivative[i] = scale * value.derivative[i];
  }
  return result;
}

Dual Cos(const Dual& value) {
  Dual result(std::cos(value.value));
  const double scale = -std::sin(value.value);
  for (std::size_t i = 0; i < kLocalVariableCount; ++i) {
    result.derivative[i] = scale * value.derivative[i];
  }
  return result;
}

Dual Sqrt(const Dual& value) {
  if (value.value == 0.0) {
    return Dual(0.0);
  }
  Dual result(std::sqrt(value.value));
  const double scale = 0.5 / result.value;
  for (std::size_t i = 0; i < kLocalVariableCount; ++i) {
    result.derivative[i] = scale * value.derivative[i];
  }
  return result;
}

Dual Hypot(const Dual& x, const Dual& y) {
  return Sqrt(x * x + y * y);
}

Dual Abs(const Dual& value) {
  if (value.value > 0.0) {
    return value;
  }
  if (value.value < 0.0) {
    return -value;
  }
  return Dual(0.0);
}

Dual Min(const Dual& lhs, const Dual& rhs) {
  return lhs.value <= rhs.value ? lhs : rhs;
}

Dual Clamp(const Dual& value, double lower, double upper) {
  if (value.value < lower) {
    return Dual(lower);
  }
  if (value.value > upper) {
    return Dual(upper);
  }
  return value;
}

Dual Atan2(const Dual& y, const Dual& x) {
  Dual result(std::atan2(y.value, x.value));
  const double denominator = x.value * x.value + y.value * y.value;
  if (denominator == 0.0) {
    return result;
  }
  for (std::size_t i = 0; i < kLocalVariableCount; ++i) {
    result.derivative[i] =
        (x.value * y.derivative[i] - y.value * x.derivative[i]) /
        denominator;
  }
  return result;
}

Dual Remainder(const Dual& value) {
  Dual result = value;
  result.value = std::remainder(value.value, kTwoPi);
  return result;
}

Dual NormalizePositiveAngle(const Dual& angle) {
  Dual normalized = angle;
  normalized.value = std::fmod(angle.value, kTwoPi);
  if (normalized.value < 0.0) {
    normalized.value += kTwoPi;
  }
  if (normalized.value >= kTwoPi - kAngleComparisonTolerance) {
    return Dual(0.0);
  }
  return normalized;
}

Dual Sinc(const Dual& angle) {
  if (std::abs(angle.value) >= kStraightAngleTolerance) {
    return Sin(angle) / angle;
  }
  const Dual squared = angle * angle;
  return Dual(1.0) - squared / Dual(6.0) +
         squared * squared / Dual(120.0);
}

Dual Cosc(const Dual& angle) {
  if (std::abs(angle.value) >= kStraightAngleTolerance) {
    return (Dual(1.0) - Cos(angle)) / angle;
  }
  const Dual squared = angle * angle;
  return angle * (Dual(0.5) - squared / Dual(24.0) +
                  squared * squared / Dual(720.0));
}

struct DualPose {
  Dual x;
  Dual y;
  Dual theta;
};

struct DualTwist {
  Dual x;
  Dual y;
  Dual theta;
};

struct DualPoint {
  Dual x;
  Dual y;
};

struct DualTrajectory {
  DualPose initial_pose;
  DualPose terminal_pose;
  DualPoint center;
  Dual radius;
  Dual swept_angle;
  bool is_straight;
};

DualPose IntegratePose(const DualPose& pose, const DualTwist& velocity,
                       double dt) {
  const Dual angle = velocity.theta * Dual(dt);
  const Dual integrated_cosine = Dual(dt) * Sinc(angle);
  const Dual integrated_sine = Dual(dt) * Cosc(angle);
  const Dual body_x =
      integrated_cosine * velocity.x - integrated_sine * velocity.y;
  const Dual body_y =
      integrated_sine * velocity.x + integrated_cosine * velocity.y;
  const Dual cos_theta = Cos(pose.theta);
  const Dual sin_theta = Sin(pose.theta);
  return {
      pose.x + cos_theta * body_x - sin_theta * body_y,
      pose.y + sin_theta * body_x + cos_theta * body_y,
      pose.theta + angle,
  };
}

DualTrajectory CalculateTrajectory(const DualPose& initial_pose,
                                   const DualTwist& velocity, double dt) {
  const Dual swept_angle = velocity.theta * Dual(dt);
  const DualPose terminal_pose = IntegratePose(initial_pose, velocity, dt);
  if (std::abs(swept_angle.value) < kStraightAngleTolerance) {
    return {initial_pose, terminal_pose, {}, Dual(0.0), swept_angle, true};
  }

  const Dual cos_theta = Cos(initial_pose.theta);
  const Dual sin_theta = Sin(initial_pose.theta);
  const Dual world_velocity_x =
      cos_theta * velocity.x - sin_theta * velocity.y;
  const Dual world_velocity_y =
      sin_theta * velocity.x + cos_theta * velocity.y;
  const DualPoint center{
      initial_pose.x - world_velocity_y / velocity.theta,
      initial_pose.y + world_velocity_x / velocity.theta,
  };
  const Dual radius =
      Hypot(velocity.x, velocity.y) / Abs(velocity.theta);
  return {initial_pose, terminal_pose, center, radius, swept_angle, false};
}

Dual DistanceToLineSegment(const DualPose& start, const DualPose& end,
                           const Point2D& point) {
  const Dual dx = end.x - start.x;
  const Dual dy = end.y - start.y;
  const Dual squared_length = dx * dx + dy * dy;
  if (squared_length.value == 0.0) {
    // This subgradient preserves a direction away from an obstacle at rest.
    return Hypot(Dual(point.x) - end.x, Dual(point.y) - end.y);
  }

  const Dual projection = Clamp(
      ((Dual(point.x) - start.x) * dx +
       (Dual(point.y) - start.y) * dy) /
          squared_length,
      0.0, 1.0);
  return Hypot(Dual(point.x) - (start.x + projection * dx),
               Dual(point.y) - (start.y + projection * dy));
}

Dual CalculateMinimumDistanceToTrajectory(
    const DualTrajectory& trajectory, const Point2D& point) {
  if (trajectory.is_straight) {
    return DistanceToLineSegment(trajectory.initial_pose,
                                 trajectory.terminal_pose, point);
  }

  Dual minimum_distance = Min(
      Hypot(Dual(point.x) - trajectory.initial_pose.x,
            Dual(point.y) - trajectory.initial_pose.y),
      Hypot(Dual(point.x) - trajectory.terminal_pose.x,
            Dual(point.y) - trajectory.terminal_pose.y));

  const double absolute_swept_angle = std::abs(trajectory.swept_angle.value);
  const Dual start_angle =
      Atan2(trajectory.initial_pose.y - trajectory.center.y,
            trajectory.initial_pose.x - trajectory.center.x);
  const Dual point_angle =
      Atan2(Dual(point.y) - trajectory.center.y,
            Dual(point.x) - trajectory.center.x);
  const Dual angle_to_point =
      trajectory.swept_angle.value > 0.0
          ? NormalizePositiveAngle(point_angle - start_angle)
          : NormalizePositiveAngle(start_angle - point_angle);
  if (absolute_swept_angle >= kTwoPi - kAngleComparisonTolerance ||
      angle_to_point.value <=
          absolute_swept_angle + kAngleComparisonTolerance) {
    minimum_distance = Min(
        minimum_distance,
        Abs(Hypot(Dual(point.x) - trajectory.center.x,
                  Dual(point.y) - trajectory.center.y) -
            trajectory.radius));
  }
  return minimum_distance;
}

Dual AngularVelocityCost(const DualTwist& velocity, const DualPose& pose,
                         const Goal& goal,
                         const GoalVelocityCostParams& params) {
  const Dual angle_error = Remainder(Dual(goal.theta) - pose.theta);
  const Dual target_angular_velocity = Clamp(
      angle_error * Dual(params.angular_distance_coefficient),
      -params.max_angular_velocity, params.max_angular_velocity);
  const Dual error = velocity.theta - target_angular_velocity;
  return Dual(params.angular_velocity_cost_coefficient) * error * error;
}

Dual CalculateGoalVelocityCost(const DualTwist& velocity,
                               const DualPose& pose, const Goal& goal,
                               const GoalVelocityCostParams& params) {
  const Dual cos_theta = Cos(pose.theta);
  const Dual sin_theta = Sin(pose.theta);
  const Dual world_velocity_x =
      cos_theta * velocity.x - sin_theta * velocity.y;
  const Dual world_velocity_y =
      sin_theta * velocity.x + cos_theta * velocity.y;

  const Dual dx = Dual(goal.x) - pose.x;
  const Dual dy = Dual(goal.y) - pose.y;
  const Dual distance = Hypot(dx, dy);
  if (distance.value <= params.position_tolerance) {
    return Dual(params.cost_coefficient) *
           (world_velocity_x * world_velocity_x +
            world_velocity_y * world_velocity_y +
            AngularVelocityCost(velocity, pose, goal, params));
  }

  const Dual goal_direction_x = dx / distance;
  const Dual goal_direction_y = dy / distance;
  const Dual goal_direction_velocity =
      goal_direction_x * world_velocity_x +
      goal_direction_y * world_velocity_y;
  const Dual lateral_velocity =
      -goal_direction_y * world_velocity_x +
      goal_direction_x * world_velocity_y;
  const Dual unconstrained_target =
      distance * Dual(params.distance_coefficient);
  const Dual target_velocity =
      unconstrained_target.value <= params.max_velocity
          ? unconstrained_target
          : Dual(params.max_velocity);
  const Dual goal_direction_velocity_error =
      goal_direction_velocity - target_velocity;

  return Dual(params.cost_coefficient) *
         (goal_direction_velocity_error * goal_direction_velocity_error +
          Dual(params.lateral_velocity_cost_coefficient) * lateral_velocity *
              lateral_velocity +
          AngularVelocityCost(velocity, pose, goal, params));
}

Dual CalculateTerminalGoalVelocityCost(
    const DualTwist& velocity, const DualPose& pose, const Goal& goal,
    const GoalVelocityCostParams& params) {
  const Dual distance =
      Hypot(Dual(goal.x) - pose.x, Dual(goal.y) - pose.y);
  double activation = 0.0;
  if (params.distance_coefficient > 0.0) {
    activation = params.max_velocity / params.distance_coefficient;
  }

  Dual proximity(0.0);
  if (distance.value <= params.position_tolerance) {
    proximity = Dual(1.0);
  } else if (activation > params.position_tolerance &&
             distance.value < activation) {
    proximity = (Dual(activation) - distance) /
                Dual(activation - params.position_tolerance);
  }

  return proximity *
         (Dual(params.terminal_velocity_cost_coefficient) *
              (velocity.x * velocity.x + velocity.y * velocity.y) +
          Dual(params.terminal_angular_velocity_cost_coefficient) *
              velocity.theta * velocity.theta);
}

struct StageDerivatives {
  std::array<double, 3> pose_cost{};
  std::array<double, 3> velocity_cost{};
  std::array<std::array<double, 3>, 3> pose_jacobian{};
  std::array<std::array<double, 3>, 3> velocity_jacobian{};
};

DualPose MakePoseVariables(const Pose2D& pose) {
  return {
      Dual::Variable(pose.x, 0),
      Dual::Variable(pose.y, 1),
      Dual::Variable(pose.theta, 2),
  };
}

DualTwist MakeVelocityVariables(const Twist2D& velocity) {
  return {
      Dual::Variable(velocity.x, 3),
      Dual::Variable(velocity.y, 4),
      Dual::Variable(velocity.theta, 5),
  };
}

Pose2D Value(const DualPose& pose) {
  return {pose.x.value, pose.y.value, pose.theta.value};
}

std::array<const Dual*, 3> Components(const DualPose& pose) {
  return {&pose.x, &pose.y, &pose.theta};
}

std::array<double*, 3> Components(Twist2D* twist) {
  return {&twist->x, &twist->y, &twist->theta};
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
  ValidateDt(dt, first_dt);
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
        ::CalculateTrajectory(pose, node.velocity, duration);
    double nearest_distance = std::numeric_limits<double>::infinity();
    for (const Point2D& point : env) {
      nearest_distance = std::min(nearest_distance,
                                  ::CalculateMinimumDistanceToTrajectory(
                                      trajectory, point));
    }

    if (std::isfinite(nearest_distance)) {
      const double clearance = nearest_distance - params_.robot_radius;
      const double penetration =
          std::max(0.0, params_.obstacle_influence_distance - clearance);
      total += params_.obstacle_cost_coefficient * penetration * penetration;
    }

    pose = trajectory.terminal_pose;
    goal_velocity_cost += ::CalculateGoalVelocityCost(
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
  total += ::CalculateTerminalGoalVelocityCost(
      terminal_node.velocity, pose, goal, params_.goal_velocity_cost);
  return total;
}

Nodes EvaluationFunction::evaluate_grad(
    const Nodes& nodes, const Pose2D& initial_pose, double dt,
    double first_dt, const Env& env, const Goal& goal) const {
  return evaluate_with_grad(nodes, initial_pose, dt, first_dt, env, goal)
      .gradient;
}

EvaluationResult EvaluationFunction::evaluate_with_grad(
    const Nodes& nodes, const Pose2D& initial_pose, double dt,
    double first_dt, const Env& env, const Goal& goal) const {
  ValidateDt(dt, first_dt);
  if (nodes.empty()) {
    return {std::numeric_limits<double>::infinity(), {}};
  }

  EvaluationResult result{0.0, Nodes(nodes.size())};
  std::vector<StageDerivatives> stages(nodes.size());
  Pose2D pose = initial_pose;
  const double goal_velocity_scale = 1.0 / static_cast<double>(nodes.size());

  for (std::size_t i = 0; i < nodes.size(); ++i) {
    const double duration = i == 0 ? first_dt : dt;
    const DualPose dual_pose = MakePoseVariables(pose);
    const DualTwist dual_velocity =
        MakeVelocityVariables(nodes[i].velocity);
    const DualTrajectory trajectory =
        CalculateTrajectory(dual_pose, dual_velocity, duration);

    Dual stage_cost(0.0);
    bool has_obstacle = false;
    Dual nearest_distance;
    for (const Point2D& point : env) {
      const Dual distance =
          CalculateMinimumDistanceToTrajectory(trajectory, point);
      if (!has_obstacle || distance.value < nearest_distance.value) {
        nearest_distance = distance;
        has_obstacle = true;
      }
    }
    if (has_obstacle) {
      const Dual clearance = nearest_distance - Dual(params_.robot_radius);
      const Dual penetration =
          Dual(params_.obstacle_influence_distance) - clearance;
      if (penetration.value > 0.0) {
        stage_cost =
            stage_cost + Dual(params_.obstacle_cost_coefficient) *
                             penetration * penetration;
      }
    }

    stage_cost =
        stage_cost +
        Dual(goal_velocity_scale) * CalculateGoalVelocityCost(
                                       dual_velocity,
                                       trajectory.terminal_pose, goal,
                                       params_.goal_velocity_cost);
    result.cost += stage_cost.value;

    StageDerivatives& stage = stages[i];
    for (std::size_t component = 0; component < 3; ++component) {
      stage.pose_cost[component] = stage_cost.derivative[component];
      stage.velocity_cost[component] =
          stage_cost.derivative[3 + component];
    }
    const auto terminal_components = Components(trajectory.terminal_pose);
    for (std::size_t output = 0; output < 3; ++output) {
      for (std::size_t input = 0; input < 3; ++input) {
        stage.pose_jacobian[output][input] =
            terminal_components[output]->derivative[input];
        stage.velocity_jacobian[output][input] =
            terminal_components[output]->derivative[3 + input];
      }
    }
    pose = Value(trajectory.terminal_pose);
  }

  for (std::size_t i = 1; i < nodes.size(); ++i) {
    const Twist2D& previous = nodes[i - 1].velocity;
    const Twist2D& current = nodes[i].velocity;
    const std::array<double, 3> difference{
        current.x - previous.x,
        current.y - previous.y,
        current.theta - previous.theta,
    };
    auto previous_gradient = Components(&result.gradient[i - 1].velocity);
    auto current_gradient = Components(&result.gradient[i].velocity);
    for (std::size_t component = 0; component < 3; ++component) {
      result.cost += params_.velocity_change_cost_coefficient *
                     difference[component] * difference[component];
      const double derivative =
          2.0 * params_.velocity_change_cost_coefficient *
          difference[component];
      *previous_gradient[component] -= derivative;
      *current_gradient[component] += derivative;
    }
  }

  const DualPose terminal_pose = MakePoseVariables(pose);
  const DualTwist terminal_velocity =
      MakeVelocityVariables(nodes.back().velocity);
  const Dual dx = terminal_pose.x - Dual(goal.x);
  const Dual dy = terminal_pose.y - Dual(goal.y);
  const Dual dtheta = Remainder(terminal_pose.theta - Dual(goal.theta));
  const Dual terminal_cost =
      Dual(params_.goal_position_cost_coefficient) * (dx * dx + dy * dy) +
      Dual(params_.goal_angle_cost_coefficient) * dtheta * dtheta +
      CalculateTerminalGoalVelocityCost(
          terminal_velocity, terminal_pose, goal,
          params_.goal_velocity_cost);
  result.cost += terminal_cost.value;

  std::array<double, 3> pose_adjoint{};
  auto terminal_gradient = Components(&result.gradient.back().velocity);
  for (std::size_t component = 0; component < 3; ++component) {
    pose_adjoint[component] = terminal_cost.derivative[component];
    *terminal_gradient[component] +=
        terminal_cost.derivative[3 + component];
  }

  for (std::size_t reverse_index = nodes.size(); reverse_index > 0;
       --reverse_index) {
    const std::size_t i = reverse_index - 1;
    const StageDerivatives& stage = stages[i];
    auto velocity_gradient = Components(&result.gradient[i].velocity);
    std::array<double, 3> previous_pose_adjoint{};
    for (std::size_t input = 0; input < 3; ++input) {
      double velocity_derivative = stage.velocity_cost[input];
      double pose_derivative = stage.pose_cost[input];
      for (std::size_t output = 0; output < 3; ++output) {
        velocity_derivative +=
            stage.velocity_jacobian[output][input] *
            pose_adjoint[output];
        pose_derivative += stage.pose_jacobian[output][input] *
                           pose_adjoint[output];
      }
      *velocity_gradient[input] += velocity_derivative;
      previous_pose_adjoint[input] = pose_derivative;
    }
    pose_adjoint = previous_pose_adjoint;
  }

  return result;
}
