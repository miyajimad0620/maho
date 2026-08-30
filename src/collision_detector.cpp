#include "maho/collision_detector.hpp"

#include <cstddef>
#include <stdexcept>

#include "maho/kinematics.hpp"
#include "trajectory.hpp"

CollisionDetector::CollisionDetector(const CollisionDetectorParams& params)
    : params_(params) {
  if (params_.robot_radius < 0.0) {
    throw std::invalid_argument("invalid collision detector parameter");
  }
}

bool CollisionDetector::detectsCollision(
    const Nodes& nodes, const Pose2D& initial_pose, double dt,
    double first_dt, const Env& env) const {
  if (!(dt > 0.0) || !(first_dt >= 0.0 && first_dt <= dt)) {
    throw std::invalid_argument("invalid collision detection dt");
  }

  Pose2D pose = initial_pose;
  for (std::size_t i = 0; i < nodes.size(); ++i) {
    const Node& node = nodes[i];
    const double duration = i == 0 ? first_dt : dt;
    const Trajectory2D trajectory =
        CalculateTrajectory(pose, node.velocity, duration);
    for (const Point2D& point : env) {
      if (CalculateMinimumDistanceToTrajectory(trajectory, point) <=
          params_.robot_radius) {
        return true;
      }
    }
    pose = trajectory.terminal_pose;
  }
  return false;
}
