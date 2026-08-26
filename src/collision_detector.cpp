#include "maho/collision_detector.hpp"

#include <cmath>
#include <stdexcept>

#include "maho/kinematics.hpp"

CollisionDetector::CollisionDetector(const CollisionDetectorParams& params)
    : params_(params) {
  if (params_.robot_radius < 0.0) {
    throw std::invalid_argument("invalid collision detector parameter");
  }
}

bool CollisionDetector::detectsCollision(
    const Nodes& nodes, const Pose2D& initial_pose, double dt,
    const Env& env) const {
  if (dt <= 0.0) {
    throw std::invalid_argument("invalid collision detection dt");
  }

  Pose2D pose = initial_pose;
  for (const Node& node : nodes) {
    pose = IntegratePose(pose, node.velocity, dt);
    for (const Point2D& point : env) {
      if (std::hypot(pose.x - point.x, pose.y - point.y) <=
          params_.robot_radius) {
        return true;
      }
    }
  }
  return false;
}
