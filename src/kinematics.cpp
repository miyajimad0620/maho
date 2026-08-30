#include "maho/kinematics.hpp"

#include <algorithm>
#include <cstddef>
#include <cmath>
#include <stdexcept>

#include "trajectory.hpp"

namespace {

constexpr double kTwoPi = 6.28318530717958647692;
constexpr double kStraightAngleTolerance = 1e-8;
constexpr double kAngleComparisonTolerance = 1e-12;

double Sinc(double angle) {
  if (std::abs(angle) >= kStraightAngleTolerance) {
    return std::sin(angle) / angle;
  }
  const double squared = angle * angle;
  return 1.0 - squared / 6.0 + squared * squared / 120.0;
}

double Cosc(double angle) {
  if (std::abs(angle) >= kStraightAngleTolerance) {
    return (1.0 - std::cos(angle)) / angle;
  }
  const double squared = angle * angle;
  return angle * (0.5 - squared / 24.0 + squared * squared / 720.0);
}

double NormalizePositiveAngle(double angle) {
  double normalized = std::fmod(angle, kTwoPi);
  if (normalized < 0.0) {
    normalized += kTwoPi;
  }
  if (normalized >= kTwoPi - kAngleComparisonTolerance) {
    return 0.0;
  }
  return normalized;
}

double DistanceToLineSegment(const Pose2D& start, const Pose2D& end,
                             const Point2D& point) {
  const double dx = end.x - start.x;
  const double dy = end.y - start.y;
  const double squared_length = dx * dx + dy * dy;
  if (squared_length == 0.0) {
    return std::hypot(point.x - start.x, point.y - start.y);
  }

  const double projection =
      std::clamp(((point.x - start.x) * dx +
                  (point.y - start.y) * dy) /
                     squared_length,
                 0.0, 1.0);
  return std::hypot(point.x - (start.x + projection * dx),
                    point.y - (start.y + projection * dy));
}

}  // namespace

Pose2D IntegratePose(const Pose2D& pose, const Twist2D& velocity,
                     double dt) {
  const double angle = velocity.theta * dt;
  const double integrated_cosine = dt * Sinc(angle);
  const double integrated_sine = dt * Cosc(angle);
  const double body_x = integrated_cosine * velocity.x -
                        integrated_sine * velocity.y;
  const double body_y = integrated_sine * velocity.x +
                        integrated_cosine * velocity.y;
  const double cos_theta = std::cos(pose.theta);
  const double sin_theta = std::sin(pose.theta);
  return {
      pose.x + cos_theta * body_x - sin_theta * body_y,
      pose.y + sin_theta * body_x + cos_theta * body_y,
      pose.theta + angle,
  };
}

Pose2D CalculateTerminalPose(const Pose2D& initial_pose,
                             const Nodes& nodes, double dt,
                             double first_dt) {
  if (!(dt > 0.0) || !(first_dt >= 0.0 && first_dt <= dt)) {
    throw std::invalid_argument("invalid trajectory dt");
  }

  Pose2D pose = initial_pose;
  for (std::size_t i = 0; i < nodes.size(); ++i) {
    const double duration = i == 0 ? first_dt : dt;
    pose = IntegratePose(pose, nodes[i].velocity, duration);
  }
  return pose;
}

Trajectory2D CalculateTrajectory(const Pose2D& initial_pose,
                                 const Twist2D& velocity, double dt) {
  const double swept_angle = velocity.theta * dt;
  const Pose2D terminal_pose = IntegratePose(initial_pose, velocity, dt);
  if (std::abs(swept_angle) < kStraightAngleTolerance) {
    return {initial_pose, terminal_pose, {}, 0.0, swept_angle, true};
  }

  const double cos_theta = std::cos(initial_pose.theta);
  const double sin_theta = std::sin(initial_pose.theta);
  const double world_velocity_x =
      cos_theta * velocity.x - sin_theta * velocity.y;
  const double world_velocity_y =
      sin_theta * velocity.x + cos_theta * velocity.y;
  const Point2D center{
      initial_pose.x - world_velocity_y / velocity.theta,
      initial_pose.y + world_velocity_x / velocity.theta,
  };
  const double radius =
      std::hypot(velocity.x, velocity.y) / std::abs(velocity.theta);
  return {initial_pose, terminal_pose, center, radius, swept_angle, false};
}

double CalculateMinimumDistanceToTrajectory(const Trajectory2D& trajectory,
                                            const Point2D& point) {
  if (trajectory.is_straight) {
    return DistanceToLineSegment(trajectory.initial_pose,
                                 trajectory.terminal_pose, point);
  }

  double minimum_distance = std::min(
      std::hypot(point.x - trajectory.initial_pose.x,
                 point.y - trajectory.initial_pose.y),
      std::hypot(point.x - trajectory.terminal_pose.x,
                 point.y - trajectory.terminal_pose.y));

  const double absolute_swept_angle = std::abs(trajectory.swept_angle);
  const double start_angle =
      std::atan2(trajectory.initial_pose.y - trajectory.center.y,
                 trajectory.initial_pose.x - trajectory.center.x);
  const double point_angle =
      std::atan2(point.y - trajectory.center.y,
                 point.x - trajectory.center.x);
  const double angle_to_point =
      trajectory.swept_angle > 0.0
          ? NormalizePositiveAngle(point_angle - start_angle)
          : NormalizePositiveAngle(start_angle - point_angle);
  if (absolute_swept_angle >= kTwoPi - kAngleComparisonTolerance ||
      angle_to_point <= absolute_swept_angle + kAngleComparisonTolerance) {
    minimum_distance = std::min(
        minimum_distance,
        std::abs(std::hypot(point.x - trajectory.center.x,
                            point.y - trajectory.center.y) -
                 trajectory.radius));
  }
  return minimum_distance;
}
